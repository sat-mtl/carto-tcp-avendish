#pragma once

/* SPDX-License-Identifier: GPL-3.0-or-later */

#include "net/NetIO.hpp"
#include "net/TripleBuffer.hpp"

#include <halp/buffer.hpp>
#include <halp/controls.hpp>
#include <halp/diagnostics.hpp>
#include <halp/meta.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <span>
#include <thread>
#include <type_traits>

namespace sat
{
/**
 * CartoTCP — receive an XYZ point cloud over TCP and expose it as a
 * TouchDesigner POP point cloud (the "P" point attribute).
 *
 * Wire protocol (byte-for-byte the pointmapper / Godot `tcp_receiver.gd`
 * reference, == Avendish's NetStream "Raw_XYZ_F32"):
 *   - uint32, little-endian: payload length in bytes (a multiple of 12)
 *   - payload: tightly-packed float32 x, y, z per point (little-endian)
 *
 * CartoTCP acts as a TCP *server*: it listens on `port`, accepts one sender,
 * and reads framed packets until the peer disconnects. The receive loop runs
 * on a background thread feeding a lock-free triple buffer; operator() (the
 * POP cook) drains the latest complete frame and uploads it. The POP binding
 * interprets the raw buffer as tightly-packed float3 positions -> "P".
 *
 * Targets are little-endian (TD: Windows x64 / macOS), matching the sender,
 * so the length prefix and the float payload are consumed without byteswap.
 */
struct CartoTCP
{
  halp_meta(name, "Carto TCP")
  halp_meta(c_name, "sat_cartotcp")
  halp_meta(category, "Network")
  halp_meta(description, "Receive an XYZ point cloud over TCP (pointmapper protocol)")
  halp_meta(author, "Société des arts technologiques")
  halp_meta(uuid, "0f2b6b89-d0cc-424b-90f3-3e2042a13b7b")

  struct
  {
    halp::spinbox_i32<"Port", halp::range{1, 65535, 9898}> port;
    halp::toggle<"Listen"> listen;
  } inputs;

  struct
  {
    // Raw CPU buffer: the POP binding reads it as tightly-packed float3 -> "P".
    halp::cpu_buffer_output<"Points"> points;

    // Routed to the Info CHOP by the POP binding.
    struct : halp::val_port<"Connected", bool>
    {
      bool value{false};
    } connected;
    struct : halp::val_port<"Point count", int>
    {
      int value{0};
    } count;
    // Frames accepted off the wire. Lets a frozen view be diagnosed from the
    // Info CHOP alone: if this keeps climbing the network side is healthy and
    // the problem is downstream, if it stops the receiver lost the stream.
    struct : halp::val_port<"Frames received", int>
    {
      int value{0};
    } frames;
  } outputs;

#ifdef CARTOTCP_DIAGNOSTICS
  // Opt-in counters for soak-testing against a real sender. Not compiled into
  // the shipped plugin; they exist so a harness can print exactly where the
  // receiver is when the stream appears to stop.
  struct debug_counters
  {
    std::atomic<uint64_t> frames{0};
    std::atomic<uint64_t> bytes{0};
    std::atomic<uint64_t> connections{0};
    std::atomic<uint64_t> rejects{0};
    std::atomic<uint32_t> last_announced{0};  // length prefix of the frame in flight
    std::atomic<uint32_t> last_reject_len{0}; // length that was refused, if any
    std::atomic<int> last_reason{0};          // see reason_* below
  } diag;

  enum reason_t
  {
    reason_none = 0,
    reason_zero_length,
    reason_oversize,
    reason_misaligned,
    reason_header_read_failed,
    reason_payload_read_failed,
  };
#endif

  // Conditions the object reports to the host. TouchDesigner turns these into
  // the node's warning/error state and the middle-click info popup.
  halp::diagnostics diagnostics;

  CartoTCP() { m_running.store(false, std::memory_order_relaxed); }
  ~CartoTCP() { stop(); }

  void operator()()
  {
    // Follow the Listen toggle, and the Port parameter: the receiver binds the
    // port it was given at start, so a change only takes effect by restarting.
    // Without this the node keeps listening on the old port while reporting the
    // new one, which points the user away from the cause.
    const bool running = m_running.load(std::memory_order_acquire);
    if(inputs.listen && running && inputs.port.value != m_active_port)
      stop();

    if(inputs.listen && !m_running.load(std::memory_order_acquire))
      start();
    else if(!inputs.listen && running)
      stop();

    // Drain the most recent complete frame produced by the I/O thread.
    if(m_buffer.consume())
    {
      const std::span<const float> src = m_buffer.read_span(); // length == 3 * N

      // The buffer descriptor points into storage, and everything below can
      // throw (create() allocates, up to the payload cap). Describe the output
      // as empty first: a throw then leaves the binding with nothing to read
      // rather than a pointer into a block that was released or never grown.
      outputs.points.buffer.raw_data = nullptr;
      outputs.points.buffer.byte_size = 0;

      try
      {
        // create() only grows, so release first when the frame is much smaller.
        auto& storage = outputs.points.storage;
        const std::size_t out_needed = src.size() * sizeof(float);
        if(storage.capacity() > min_retained_bytes
           && storage.capacity() > shrink_threshold * out_needed)
          std::remove_reference_t<decltype(storage)>{}.swap(storage);

        const std::span<float> dst = outputs.points.create<float>(
            static_cast<int64_t>(src.size()));
        std::copy(src.begin(), src.end(), dst.begin());
        outputs.points.upload();
        outputs.count.value = static_cast<int>(src.size() / 3);
      }
      catch(...)
      {
        // Out of memory for this cloud. The descriptor is already empty, so the
        // node shows nothing this cook instead of taking the host down.
        outputs.points.buffer.raw_data = nullptr;
        outputs.points.buffer.byte_size = 0;
        outputs.count.value = 0;
        m_alloc_failures.fetch_add(1, std::memory_order_relaxed);
      }
    }

    // The binding clears `changed` after writing and then emits nothing at all
    // on a cook where it is false. Re-assert, so cooks landing between two
    // packets keep the current cloud instead of blanking.
    if(outputs.points.buffer.raw_data && outputs.points.buffer.byte_size > 0)
      outputs.points.upload();

    outputs.connected.value = m_connected.load(std::memory_order_acquire);
    outputs.frames.value
        = static_cast<int>(m_frames.load(std::memory_order_relaxed) & 0x7FFFFFFF);

    report_state();
  }

private:
  // Diagnostics are raised here, on the cook thread, from state the receiver
  // published. They are cleared by the binding before each cook, so everything
  // that still applies has to be restated.
  void report_state()
  {
    const int port = inputs.port.value;

    if(!inputs.listen)
    {
      diagnostics.info<"idle">("not listening - enable Listen");
      return;
    }

    const bool connected = m_connected.load(std::memory_order_acquire);
    const auto [f, detail] = current_fault();

    switch(f)
    {
      case fault::bind_failed:
        diagnostics.error<"port_unavailable">(
            "cannot listen on port {} - already in use by another program or "
            "another copy of this node",
            port);
        break;
      case fault::bad_length:
        diagnostics.warning<"bad_frame">(
            "sender announced a {}-byte frame, which is not a valid point cloud - "
            "dropped the connection to resynchronise",
            detail);
        break;
      case fault::peer_stalled:
        diagnostics.warning<"sender_stalled">(
            "sender stopped part-way through a {}-byte frame - reconnecting", detail);
        break;
      case fault::peer_replaced:
        diagnostics.info<"reconnected">(
            "the sender reconnected, switched to the new connection");
        break;
      case fault::internal_error:
        diagnostics.error<"internal_error">(
            "the receiver stopped unexpectedly, retrying");
        break;
      case fault::peer_disconnected:
      case fault::none:
        break;
    }

    if(const uint32_t oom = m_alloc_failures.load(std::memory_order_relaxed); oom > 0)
    {
      diagnostics.error<"out_of_memory">(
          "{} cloud(s) were too large to allocate and were dropped", oom);
    }

    if(connected)
    {
      // Silence is reported but never acted on: dropping a quiet sender would
      // strand one that still thinks it is connected.
      if(const double quiet = seconds_since_frame(); quiet > silence_warn_seconds)
        diagnostics.warning<"sender_silent">(
            "connected, but no data for {:.0f}s", quiet);

      diagnostics.info<"streaming">(
          "{} points, {} frames received", outputs.count.value, outputs.frames.value);
    }
    else if(f == fault::none || f == fault::peer_disconnected)
    {
      diagnostics.warning<"no_sender">(
          "listening on port {}, no sender connected", port);
    }

    if(const uint32_t resyncs = m_resyncs.load(std::memory_order_relaxed); resyncs > 0)
    {
      diagnostics.info<"resyncs">(
          "{} connection(s) dropped so far to resynchronise the stream", resyncs);
    }
  }

private:
  void start()
  {
    if(m_running.load(std::memory_order_acquire))
      return;

    // Reap a receiver that returned on its own: std::thread::operator=
    // terminates if the target is still joinable.
    if(m_thread.joinable())
      m_thread.join();

    m_running.store(true, std::memory_order_release);
    m_active_port = inputs.port.value;
    try
    {
      // Snapshot the port: the parameter belongs to the cook thread.
      m_thread = std::thread([this, port = m_active_port] { receiver_thread(port); });
    }
    catch(...)
    {
      // Thread creation failed: roll back so the next cook retries, and never
      // let the exception escape into the host.
      m_running.store(false, std::memory_order_release);
    }
  }

  // Runs on the cook thread, so it must return promptly: every wait in the
  // receiver is bounded by poll_timeout_ms. Touches no socket -- those belong
  // to the receiver thread.
  void stop()
  {
    m_running.store(false, std::memory_order_release);
    if(m_thread.joinable())
      m_thread.join();
    m_connected.store(false, std::memory_order_release);
    // Toggling Listen is how a user retries after fixing something, so it must
    // not carry the previous failure over to the next attempt.
    set_fault(fault::none);
  }

  void receiver_thread(int port)
  {
    using namespace netstream;

    // m_running must end up false on every exit path, or operator() sees a live
    // receiver forever and never restarts it.
    struct scope_exit
    {
      CartoTCP& self;
      ~scope_exit()
      {
        self.m_connected.store(false, std::memory_order_release);
        self.m_running.store(false, std::memory_order_release);
      }
    } on_exit{*this};

    try
    {
      while(m_running.load(std::memory_order_acquire))
      {
        // Owned here, not in a member: the cook thread never sees this handle.
        Socket listen_socket = create_tcp_socket();
        bool bound = false;
        if(listen_socket.is_valid())
        {
          set_exclusive_addr(listen_socket.native_handle());
          bound = bind_and_listen(listen_socket.native_handle(), port, listen_backlog);
        }

        if(!bound)
        {
          // Port taken, or out of descriptors: retry so the POP recovers by
          // itself once it frees.
          set_fault(fault::bind_failed, static_cast<uint32_t>(port));
          sleep_while_running(retry_delay_ms);
          continue;
        }

        // Bound: whatever went wrong before no longer applies. Clearing only
        // once a peer connects would leave a stale "port in use" on a node that
        // is now listening perfectly well.
        set_fault(fault::none);

        set_non_blocking(listen_socket.native_handle());
        serve(listen_socket);
      }
    }
    catch(...)
    {
      // A dead receiver just means no new frames; start() can spawn another.
      // Say so, and back off: operator() restarts it on the very next cook, so
      // a persistent failure would otherwise churn a thread per frame.
      set_fault(fault::internal_error);
      sleep_while_running(retry_delay_ms);
    }
  }

  // Accept peers one at a time and stream from each until it disconnects.
  void serve(netstream::Socket& listen_socket)
  {
    using namespace netstream;

    while(m_running.load(std::memory_order_acquire))
    {
      // Non-blocking accept, polled in short slices so Listen-off stays responsive.
      Socket client;
      int consecutive_errors = 0;
      while(m_running.load(std::memory_order_acquire))
      {
        client = accept_connection(listen_socket.native_handle());
        if(client.is_valid())
          break;

        // Most accept errors are per-connection or transient, and rebuilding the
        // listener fixes neither, so only rebuild once they look persistent.
        if(would_block())
          consecutive_errors = 0;
        else if(++consecutive_errors > max_accept_errors)
          return; // listener looks unusable: rebuild it

        std::this_thread::sleep_for(std::chrono::milliseconds(poll_timeout_ms));
      }
      if(!client.is_valid())
        return;

      configure_client(client);

      m_connected.store(true, std::memory_order_release);
      set_fault(fault::none);
      mark_frame_time();
#ifdef CARTOTCP_DIAGNOSTICS
      diag.connections.fetch_add(1, std::memory_order_relaxed);
#endif

      while(m_running.load(std::memory_order_acquire))
      {
        if(!receive_frame(client, listen_socket))
          break;
        refresh_quickack(client.native_handle());
      }

      m_connected.store(false, std::memory_order_release);
    }
  }

  // Wait, in slices short enough that stop() stays responsive.
  void sleep_while_running(int total_ms)
  {
    for(int waited = 0; waited < total_ms; waited += poll_timeout_ms)
    {
      if(!m_running.load(std::memory_order_acquire))
        return;
      std::this_thread::sleep_for(std::chrono::milliseconds(poll_timeout_ms));
    }
  }

  void configure_client(netstream::Socket& client)
  {
    using namespace netstream;
    // Blocking with a receive timeout, so a peer that goes quiet mid-frame
    // cannot pin this thread.
    set_blocking(client.native_handle());
    set_recv_timeout(client.native_handle(), poll_timeout_ms);
    optimize_for_low_latency(client.native_handle());
  }

  // Wait for the next frame's header.
  //
  // A quiet sender is not an error and must not be disconnected: it may simply
  // have nothing to send, and dropping it strands a peer that still believes the
  // connection is live -- it would resume writing into a socket we closed. So
  // this waits indefinitely, and instead watches for a *replacement* connection,
  // which is the one reliable sign the current one is finished: on a half-open
  // socket, an idle peer and a dead peer are indistinguishable.
  bool await_header(
      netstream::Socket& client, netstream::Socket& listen_socket,
      uint32_t& payload_bytes)
  {
    using namespace netstream;

    std::span<uint8_t> len_span(
        reinterpret_cast<uint8_t*>(&payload_bytes), sizeof(payload_bytes));
    std::size_t got = 0;
    std::chrono::steady_clock::time_point partial_since{};

    while(m_running.load(std::memory_order_acquire))
    {
      switch(recv_some_until(
          client.native_handle(), len_span, got, m_running, header_poll))
      {
        case recv_status::complete:
          return true;
        case recv_status::failed:
          if(m_running.load(std::memory_order_acquire))
            set_fault(fault::peer_disconnected);
          return false;
        case recv_status::timed_out:
          break;
      }

      // Only swap while no part of a header is buffered, or those bytes would
      // be lost and the framing would desynchronise.
      if(got == 0)
      {
        partial_since = {};

        Socket replacement = accept_connection(listen_socket.native_handle());
        if(replacement.is_valid())
        {
          configure_client(replacement);
          client = std::move(replacement);
          set_fault(fault::peer_replaced);
          m_resyncs.fetch_add(1, std::memory_order_relaxed);
          mark_frame_time();
        }
        continue;
      }

      // A header stuck part-way is the same stall the payload path treats as
      // fatal, and while it lasts the swap above cannot run -- so a peer that
      // reconnects would never be accepted. Give it the same deadline.
      if(partial_since == std::chrono::steady_clock::time_point{})
        partial_since = std::chrono::steady_clock::now();
      else if(std::chrono::steady_clock::now() - partial_since > frame_limit)
      {
        set_fault(fault::peer_stalled, static_cast<uint32_t>(got));
        m_resyncs.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
    }
    return false;
  }

  // Read one length-prefixed XYZ-f32 frame straight into the write buffer.
  bool receive_frame(netstream::Socket& client, netstream::Socket& listen_socket)
  {
    using namespace netstream;

    // 4-byte little-endian payload length (in bytes).
    uint32_t payload_bytes = 0;
    if(!await_header(client, listen_socket, payload_bytes))
    {
#ifdef CARTOTCP_DIAGNOSTICS
      diag.last_reason.store(reason_header_read_failed, std::memory_order_relaxed);
#endif
      return false;
    }
#ifdef CARTOTCP_DIAGNOSTICS
    diag.last_announced.store(payload_bytes, std::memory_order_relaxed);
    diag.bytes.fetch_add(sizeof(payload_bytes), std::memory_order_relaxed);
#endif

    // An empty cloud is legitimate: carto streams zero-length frames at the full
    // rate whenever the sensor sees nothing.
    if(payload_bytes == 0)
    {
      auto& write_buf = m_buffer.write_buffer();
      write_buf.clear(); // keep capacity, the scene usually refills
      m_buffer.publish();
      m_frames.fetch_add(1, std::memory_order_relaxed);
      mark_frame_time();
#ifdef CARTOTCP_DIAGNOSTICS
      diag.frames.fetch_add(1, std::memory_order_relaxed);
#endif
      return true;
    }

    // The announced length is allocated in full before any payload arrives, so
    // it has to be bounded. Past the cap the stream is desynced: drop the
    // connection and resynchronise on a fresh one.
    constexpr uint32_t point_bytes = 3 * sizeof(float); // 12
    if(payload_bytes > max_payload_bytes || (payload_bytes % point_bytes) != 0)
    {
      set_fault(fault::bad_length, payload_bytes);
      m_resyncs.fetch_add(1, std::memory_order_relaxed);
#ifdef CARTOTCP_DIAGNOSTICS
      diag.rejects.fetch_add(1, std::memory_order_relaxed);
      diag.last_reject_len.store(payload_bytes, std::memory_order_relaxed);
      diag.last_reason.store(
          payload_bytes % point_bytes ? reason_misaligned : reason_oversize,
          std::memory_order_relaxed);
#endif
      return false;
    }

    const std::size_t num_floats = payload_bytes / sizeof(float);
    auto& write_buf = m_buffer.write_buffer();

    // Release capacity when frames shrink a lot: the three slots rotate, so one
    // outsized cloud would otherwise pin 3x its size for the node's lifetime.
    // Contents are about to be overwritten, so drop the block rather than pay
    // shrink_to_fit()'s copy.
    using buffer_type = std::remove_reference_t<decltype(write_buf)>;
    const std::size_t cap_bytes = write_buf.capacity() * sizeof(float);
    if(cap_bytes > min_retained_bytes && cap_bytes > shrink_threshold * payload_bytes)
      buffer_type{}.swap(write_buf);

    write_buf.resize(num_floats);

    std::span<uint8_t> dst(
        reinterpret_cast<uint8_t*>(write_buf.data()), payload_bytes);
    if(!recv_all_until(client.native_handle(), dst, m_running, frame_limit))
    {
      // Committed to a frame the peer never finished sending.
      if(m_running.load(std::memory_order_acquire))
      {
        set_fault(fault::peer_stalled, payload_bytes);
        m_resyncs.fetch_add(1, std::memory_order_relaxed);
      }
#ifdef CARTOTCP_DIAGNOSTICS
      diag.last_reason.store(reason_payload_read_failed, std::memory_order_relaxed);
#endif
      return false;
    }
    m_frames.fetch_add(1, std::memory_order_relaxed);
    mark_frame_time();
#ifdef CARTOTCP_DIAGNOSTICS
    diag.frames.fetch_add(1, std::memory_order_relaxed);
    diag.bytes.fetch_add(payload_bytes, std::memory_order_relaxed);
#endif

    m_buffer.publish();
    return true;
  }

  // Longest the receiver can be mid-wait when stop() asks it to quit, so also
  // the worst-case cost of Listen-off / node deletion on the cook thread.
  static constexpr int poll_timeout_ms = 20;

  // How long to wait before retrying a listening socket that would not bind.
  static constexpr int retry_delay_ms = 1000;

  // Consecutive accept() failures tolerated before the listener is rebuilt,
  // ~2 s at poll_timeout_ms apiece.
  static constexpr int max_accept_errors = 100;

  // How long to wait for a frame header before coming up for air to check
  // whether another peer is trying to connect. Not a disconnect timeout.
  static constexpr auto header_poll = std::chrono::milliseconds(250);

  // Silence beyond this is reported to the user, but the connection is kept:
  // a sender with nothing to say is not a broken one.
  static constexpr double silence_warn_seconds = 5.0;

  // Budget for one frame once its header is accepted. Real frames transfer in
  // tens of milliseconds, so reaching this means the length prefix was wrong and
  // the frame would never complete.
  static constexpr auto frame_limit = std::chrono::seconds(15);

  // Room for a peer to reconnect while the previous socket is still being read.
  static constexpr int listen_backlog = 8;

  // Release a buffer holding this many times more than the frame needs, but
  // never below the floor: ordinary frame-to-frame variation must not realloc.
  static constexpr std::size_t shrink_threshold = 4;
  static constexpr std::size_t min_retained_bytes = 4u * 1024u * 1024u;

  // Largest cloud accepted; past this the length prefix is a desync. Production
  // frames are ~1 M points, the headroom is for deliberately large tests.
  static constexpr uint32_t max_points = 16'000'000;
  static constexpr uint32_t max_payload_bytes = max_points * 3u * sizeof(float);
  static_assert(max_payload_bytes < INT32_MAX && max_payload_bytes % 12 == 0);

  // What the receiver is currently unhappy about. Set on the receive thread,
  // read by the cook, which is the thread allowed to raise diagnostics.
  enum class fault : int
  {
    none = 0,
    bind_failed,       // port taken, or out of descriptors
    bad_length,        // announced size is not a plausible frame: desync
    peer_stalled,      // committed to a frame the peer never finished
    peer_replaced,     // a new connection took over from a dead-looking one
    peer_disconnected, // clean close or reset
    internal_error     // the receiver threw
  };

  // Kind and detail travel together in one word: read separately, the cook
  // could pair one fault with another's detail and report a nonsense number.
  std::atomic<uint64_t> m_fault_state{0};
  std::atomic<uint32_t> m_resyncs{0};       // connections replaced or resynchronised
  std::atomic<uint32_t> m_alloc_failures{0}; // clouds too big to allocate
  int m_active_port{0};                      // cook thread only
  std::atomic<int64_t> m_last_frame_ns{0}; // steady_clock, for reporting silence

  void mark_frame_time() noexcept
  {
    m_last_frame_ns.store(
        std::chrono::steady_clock::now().time_since_epoch().count(),
        std::memory_order_relaxed);
  }

  // Seconds since the last frame, or -1 if none has arrived yet.
  double seconds_since_frame() const noexcept
  {
    const int64_t then = m_last_frame_ns.load(std::memory_order_relaxed);
    if(then == 0)
      return -1.0;
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::chrono::duration<double>(
               std::chrono::steady_clock::duration(now - then))
        .count();
  }

  void set_fault(fault f, uint32_t detail = 0) noexcept
  {
    const uint64_t packed
        = (static_cast<uint64_t>(static_cast<uint32_t>(f)) << 32) | detail;
    m_fault_state.store(packed, std::memory_order_release);
  }

  std::pair<fault, uint32_t> current_fault() const noexcept
  {
    const uint64_t packed = m_fault_state.load(std::memory_order_acquire);
    return {static_cast<fault>(static_cast<uint32_t>(packed >> 32)),
            static_cast<uint32_t>(packed)};
  }

  std::atomic<bool> m_running{false};
  std::atomic<bool> m_connected{false};
  std::atomic<uint64_t> m_frames{0};
  std::thread m_thread;
  netstream::TripleBuffer<float> m_buffer;
};
}
