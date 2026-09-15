/* SPDX-License-Identifier: GPL-3.0-or-later */

/**
 * fake_cloud_sender — synthetic point-cloud source for the Carto TCP POP.
 *
 * Speaks the exact same wire protocol as the real carto sender (see
 * src/CartoTCP.hpp): TCP client, and per frame
 *   - uint32 little-endian: payload length in bytes (multiple of 12)
 *   - payload: tightly-packed float32 x,y,z per point
 *
 * The POP is the *server*: set Port + Listen in TouchDesigner first, then run
 * this. Beyond the nominal stream it can reproduce the misbehaviours a real
 * sensor/network exhibits (varying point counts, reconnects, half-sent frames,
 * stalls, garbage lengths), which is what makes TouchDesigner hang or crash.
 *
 * Build (standalone, no Avendish needed):
 *   cl /std:c++20 /EHsc /Fe:fake_cloud_sender.exe tools\fake_cloud_sender.cpp ^
 *      /I src ws2_32.lib
 *   c++ -std=c++20 -O2 -I src -o fake_cloud_sender tools/fake_cloud_sender.cpp
 * or via CMake: -DCARTOTCP_BUILD_TOOLS=ON (target: fake_cloud_sender).
 */

#include <net/NetIO.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numbers>
#include <random>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <consoleapi.h>
#else
#include <csignal>
#endif

namespace
{
using clock_type = std::chrono::steady_clock;

std::atomic<bool> g_quit{false};

struct Options
{
  std::string host = "127.0.0.1";
  int port = 9898;

  int points = 65536;    // points per frame (when not varying)
  int points_min = 0;    // --vary MIN:MAX -> random count in [min, max]
  int points_max = 0;
  double fps = 60.0;     // 0 => send as fast as the socket accepts
  double duration = 0.0; // seconds, 0 => until Ctrl-C
  unsigned seed = 1234;

  enum class Pattern
  {
    sphere,
    wave,
    cube
  } pattern = Pattern::sphere;

  // Sender-side misbehaviours, all off by default.
  double reconnect = 0.0; // close + reconnect every N seconds
  double stall = 0.0;     // every stall_every frames: send the header, then
  int stall_every = 0;    //   wait `stall` seconds before the payload
  int truncate_every = 0; // send half a payload, then close the connection
  int badlen_every = 0;   // length prefix that is not a multiple of 12
  int huge_every = 0;     // absurd length prefix (~2 GB) with no payload
  bool split_header = false; // separate write() for header and payload
};

[[noreturn]] void usage(int code)
{
  std::fputs(
      "fake_cloud_sender — stream a synthetic XYZ point cloud to the Carto TCP POP\n"
      "\n"
      "Usage: fake_cloud_sender [options]\n"
      "\n"
      "Stream:\n"
      "  --host <addr>       POP host (default 127.0.0.1)\n"
      "  --port <n>          POP port (default 9898)\n"
      "  --points <n>        points per frame (default 65536)\n"
      "  --vary <min:max>    random point count per frame instead of --points\n"
      "  --fps <f>           frames per second, 0 = flat out (default 60)\n"
      "  --duration <s>      stop after s seconds (default: run until Ctrl-C)\n"
      "  --pattern <p>       sphere | wave | cube (default sphere)\n"
      "  --seed <n>          RNG seed (default 1234)\n"
      "\n"
      "Fault injection (off unless asked for):\n"
      "  --reconnect <s>     drop and reopen the connection every s seconds\n"
      "  --stall <s>[:<n>]   every n frames (default 1), send the length prefix\n"
      "                      then wait s seconds before the payload\n"
      "  --truncate <n>      every n frames, send half the payload and hang up\n"
      "  --badlen <n>        every n frames, send a length that is not a\n"
      "                      multiple of 12 (protocol violation)\n"
      "  --huge <n>          every n frames, announce a ~2 GB payload\n"
      "  --split-header      write the length prefix and payload separately\n"
      "  -h, --help          this help\n",
      code == 0 ? stdout : stderr);
  std::exit(code);
}

const char* need_value(int argc, char** argv, int& i)
{
  if(i + 1 >= argc)
  {
    std::fprintf(stderr, "error: %s needs a value\n", argv[i]);
    usage(2);
  }
  return argv[++i];
}

Options parse_args(int argc, char** argv)
{
  Options o;
  for(int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    if(a == "-h" || a == "--help")
      usage(0);
    else if(a == "--host")
      o.host = need_value(argc, argv, i);
    else if(a == "--port")
      o.port = std::atoi(need_value(argc, argv, i));
    else if(a == "--points")
      o.points = std::atoi(need_value(argc, argv, i));
    else if(a == "--vary")
    {
      const std::string v = need_value(argc, argv, i);
      const auto colon = v.find(':');
      if(colon == std::string::npos)
        usage(2);
      o.points_min = std::atoi(v.substr(0, colon).c_str());
      o.points_max = std::atoi(v.substr(colon + 1).c_str());
    }
    else if(a == "--fps")
      o.fps = std::atof(need_value(argc, argv, i));
    else if(a == "--duration")
      o.duration = std::atof(need_value(argc, argv, i));
    else if(a == "--seed")
      o.seed = static_cast<unsigned>(std::atoi(need_value(argc, argv, i)));
    else if(a == "--pattern")
    {
      const std::string p = need_value(argc, argv, i);
      if(p == "sphere")
        o.pattern = Options::Pattern::sphere;
      else if(p == "wave")
        o.pattern = Options::Pattern::wave;
      else if(p == "cube")
        o.pattern = Options::Pattern::cube;
      else
        usage(2);
    }
    else if(a == "--reconnect")
      o.reconnect = std::atof(need_value(argc, argv, i));
    else if(a == "--stall")
    {
      const std::string v = need_value(argc, argv, i);
      const auto colon = v.find(':');
      o.stall = std::atof(v.substr(0, colon).c_str());
      o.stall_every
          = colon == std::string::npos ? 1 : std::atoi(v.substr(colon + 1).c_str());
    }
    else if(a == "--truncate")
      o.truncate_every = std::atoi(need_value(argc, argv, i));
    else if(a == "--badlen")
      o.badlen_every = std::atoi(need_value(argc, argv, i));
    else if(a == "--huge")
      o.huge_every = std::atoi(need_value(argc, argv, i));
    else if(a == "--split-header")
      o.split_header = true;
    else
    {
      std::fprintf(stderr, "error: unknown option '%s'\n", a.c_str());
      usage(2);
    }
  }

  if(o.points_max > 0)
  {
    if(o.points_min < 1 || o.points_min > o.points_max)
    {
      std::fputs("error: --vary needs 1 <= min <= max\n", stderr);
      usage(2);
    }
  }
  else if(o.points < 1)
  {
    std::fputs("error: --points must be >= 1\n", stderr);
    usage(2);
  }
  if(o.port < 1 || o.port > 65535)
  {
    std::fputs("error: --port out of range\n", stderr);
    usage(2);
  }
  return o;
}

// Fill `dst` (3 * count floats) with one animated frame.
void generate(std::span<float> dst, Options::Pattern pattern, double t, std::mt19937& rng)
{
  const std::size_t count = dst.size() / 3;
  const auto ft = static_cast<float>(t);

  switch(pattern)
  {
    case Options::Pattern::sphere:
    {
      // Fibonacci sphere, radius pulsing over time: dense and easy to eyeball.
      constexpr float golden = 2.399963f; // pi * (3 - sqrt(5))
      const float radius = 1.f + 0.25f * std::sin(ft * 1.7f);
      for(std::size_t i = 0; i < count; ++i)
      {
        const float f = (2.f * static_cast<float>(i)) / static_cast<float>(count) - 1.f;
        const float y = f;
        const float r = std::sqrt(std::max(0.f, 1.f - y * y)) * radius;
        const float theta = golden * static_cast<float>(i) + ft;
        dst[i * 3 + 0] = r * std::cos(theta);
        dst[i * 3 + 1] = y * radius;
        dst[i * 3 + 2] = r * std::sin(theta);
      }
      break;
    }
    case Options::Pattern::wave:
    {
      // Square grid with a travelling ripple.
      const auto side = static_cast<std::size_t>(
          std::max(1.0, std::sqrt(static_cast<double>(count))));
      for(std::size_t i = 0; i < count; ++i)
      {
        const float u = static_cast<float>(i % side) / static_cast<float>(side) - 0.5f;
        const float v = static_cast<float>(i / side) / static_cast<float>(side) - 0.5f;
        dst[i * 3 + 0] = u * 2.f;
        dst[i * 3 + 1] = 0.3f * std::sin(8.f * std::sqrt(u * u + v * v) - ft * 3.f);
        dst[i * 3 + 2] = v * 2.f;
      }
      break;
    }
    case Options::Pattern::cube:
    {
      // Uniform noise in a unit cube — worst case for any spatial structure.
      std::uniform_real_distribution<float> d(-1.f, 1.f);
      for(std::size_t i = 0; i < count * 3; ++i)
        dst[i] = d(rng);
      break;
    }
  }
}

void install_signal_handler()
{
#if defined(_WIN32)
  SetConsoleCtrlHandler(
      [](DWORD) -> BOOL
      {
        g_quit.store(true);
        return TRUE;
      },
      TRUE);
#else
  std::signal(SIGINT, [](int) { g_quit.store(true); });
  std::signal(SIGPIPE, SIG_IGN);
#endif
}
}

int main(int argc, char** argv)
{
  const Options opt = parse_args(argc, argv);
  install_signal_handler();

  std::mt19937 rng{opt.seed};
  std::uniform_int_distribution<int> count_dist{opt.points_min, opt.points_max};

  // One contiguous buffer: [uint32 length][payload]. A real sender does the
  // same, so this exercises the receiver's framing exactly as in production.
  std::vector<unsigned char> frame;

  const auto t0 = clock_type::now();
  const auto elapsed = [&t0] {
    return std::chrono::duration<double>(clock_type::now() - t0).count();
  };

  std::printf(
      "fake_cloud_sender -> %s:%d | %s | %s pts/frame | %.3g fps\n", opt.host.c_str(),
      opt.port,
      opt.pattern == Options::Pattern::sphere
          ? "sphere"
          : (opt.pattern == Options::Pattern::wave ? "wave" : "cube"),
      opt.points_max > 0
          ? (std::to_string(opt.points_min) + ".." + std::to_string(opt.points_max))
                .c_str()
          : std::to_string(opt.points).c_str(),
      opt.fps);
  std::fflush(stdout);

  uint64_t total_frames = 0;
  uint64_t total_bytes = 0;
  uint64_t window_frames = 0;
  uint64_t window_bytes = 0;
  double last_report = 0.0;
  int connections = 0;

  while(!g_quit.load() && (opt.duration <= 0.0 || elapsed() < opt.duration))
  {
    // ---- connect -------------------------------------------------------
    netstream::Socket sock = netstream::create_tcp_socket();
    if(!sock.is_valid())
    {
      std::fprintf(stderr, "socket() failed (%d)\n", netstream::get_socket_error());
      return 1;
    }
    if(!netstream::connect_socket(sock.native_handle(), opt.host, opt.port))
    {
      std::fprintf(
          stderr, "waiting for %s:%d (is the POP listening?)\n", opt.host.c_str(),
          opt.port);
      sock.close();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }
    netstream::optimize_for_low_latency(sock.native_handle());
    std::printf("connected (#%d) at t=%.1fs\n", ++connections, elapsed());
    std::fflush(stdout);

    const double connected_at = elapsed();
    auto next_frame = clock_type::now();
    bool peer_gone = false;

    // ---- stream --------------------------------------------------------
    while(!g_quit.load() && !peer_gone)
    {
      const double now = elapsed();
      if(opt.duration > 0.0 && now >= opt.duration)
        break;
      if(opt.reconnect > 0.0 && now - connected_at >= opt.reconnect)
      {
        std::printf("dropping connection after %.1fs\n", now - connected_at);
        std::fflush(stdout);
        break;
      }

      const int count = opt.points_max > 0 ? count_dist(rng) : opt.points;
      const auto payload_bytes = static_cast<uint32_t>(count) * 12u;

      frame.resize(4u + payload_bytes);
      uint32_t announced = payload_bytes;

      // Never on the very first frame: let a good cloud land before misbehaving.
      const auto every
          = [&](int n) { return n > 0 && total_frames > 0 && (total_frames % n) == 0; };
      const bool bad_len = every(opt.badlen_every);
      const bool huge = every(opt.huge_every);
      const bool truncate = every(opt.truncate_every);
      const bool stalled = opt.stall > 0.0 && every(opt.stall_every);

      if(huge)
        announced = 2147483640u; // 12 * 178956970, just under INT32_MAX: passes
                                 // the receiver's validation and forces a 2 GB alloc
      else if(bad_len)
        announced = payload_bytes + 5u; // not a multiple of 12

      std::memcpy(frame.data(), &announced, sizeof(announced)); // little-endian host
      generate(
          {reinterpret_cast<float*>(frame.data() + 4), static_cast<std::size_t>(count) * 3},
          opt.pattern, now, rng);

      std::size_t to_send = frame.size();
      if(huge)
        to_send = 4; // announce only, then let the receiver wait
      else if(truncate)
        to_send = 4 + payload_bytes / 2;

      bool ok = true;
      if(opt.split_header || stalled)
      {
        ok = netstream::send_all(sock.native_handle(), {frame.data(), 4});
        if(stalled && ok)
        {
          std::printf("stalling %.2fs mid-frame\n", opt.stall);
          std::fflush(stdout);
          std::this_thread::sleep_for(
              std::chrono::duration<double>(opt.stall));
        }
        if(ok && to_send > 4)
          ok = netstream::send_all(
              sock.native_handle(), {frame.data() + 4, to_send - 4});
      }
      else
      {
        ok = netstream::send_all(sock.native_handle(), {frame.data(), to_send});
      }

      if(!ok)
      {
        std::printf(
            "peer closed the connection (send failed, err %d)\n",
            netstream::get_socket_error());
        std::fflush(stdout);
        peer_gone = true;
        break;
      }

      ++total_frames;
      ++window_frames;
      total_bytes += to_send;
      window_bytes += to_send;

      if(truncate)
      {
        std::printf("sent a truncated frame, hanging up\n");
        std::fflush(stdout);
        break;
      }
      if(huge)
      {
        std::printf("announced a %u-byte payload with no data\n", announced);
        std::fflush(stdout);
        // Keep the connection open: the receiver is now blocked in recv().
        std::this_thread::sleep_for(std::chrono::seconds(2));
        break;
      }
      if(bad_len)
      {
        std::printf("sent a misaligned length (%u bytes)\n", announced);
        std::fflush(stdout);
      }

      if(now - last_report >= 1.0)
      {
        std::printf(
            "  t=%6.1fs  %5.1f fps  %6.1f MB/s  %llu frames total\n", now,
            window_frames / (now - last_report),
            window_bytes / (now - last_report) / (1024.0 * 1024.0),
            static_cast<unsigned long long>(total_frames));
        std::fflush(stdout);
        last_report = now;
        window_frames = 0;
        window_bytes = 0;
      }

      if(opt.fps > 0.0)
      {
        next_frame += std::chrono::duration_cast<clock_type::duration>(
            std::chrono::duration<double>(1.0 / opt.fps));
        const auto now_tp = clock_type::now();
        if(next_frame > now_tp)
          std::this_thread::sleep_until(next_frame);
        else
          next_frame = now_tp; // fell behind: don't accumulate debt
      }
    }

    sock.close();
    if(!g_quit.load() && (opt.duration <= 0.0 || elapsed() < opt.duration))
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::printf(
      "\nstopped: %llu frames, %.1f MB, %d connection(s) in %.1fs\n",
      static_cast<unsigned long long>(total_frames),
      total_bytes / (1024.0 * 1024.0), connections, elapsed());
  return 0;
}
