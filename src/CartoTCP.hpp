#pragma once

/* SPDX-License-Identifier: GPL-3.0-or-later */

#include "net/NetIO.hpp"
#include "net/TripleBuffer.hpp"

#include <halp/buffer.hpp>
#include <halp/controls.hpp>
#include <halp/meta.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <span>
#include <thread>

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
  } outputs;

  CartoTCP() { m_running.store(false, std::memory_order_relaxed); }
  ~CartoTCP() { stop(); }

  void operator()()
  {
    // Follow the Listen toggle: start / stop the background listener.
    if(inputs.listen && !m_running.load(std::memory_order_acquire))
      start();
    else if(!inputs.listen && m_running.load(std::memory_order_acquire))
      stop();

    // Drain the most recent complete frame produced by the I/O thread.
    if(m_buffer.consume())
    {
      const std::span<const float> src = m_buffer.read_span(); // length == 3 * N
      const std::span<float> dst = outputs.points.create<float>(
          static_cast<int64_t>(src.size()));
      std::copy(src.begin(), src.end(), dst.begin());
      outputs.points.upload();
      outputs.count.value = static_cast<int>(src.size() / 3);
    }

    outputs.connected.value = m_connected.load(std::memory_order_acquire);
  }

private:
  void start()
  {
    if(m_running.load(std::memory_order_acquire))
      return;
    m_running.store(true, std::memory_order_release);
    m_thread = std::thread([this] { receiver_thread(); });
  }

  void stop()
  {
    m_running.store(false, std::memory_order_release);
    if(m_thread.joinable())
    {
      // Unblock any accept()/recv() stuck in the I/O thread, then join.
      m_listen_socket.shutdown();
      m_client_socket.shutdown();
      m_thread.join();
      m_listen_socket.close();
      m_client_socket.close();
    }
    m_connected.store(false, std::memory_order_release);
  }

  void receiver_thread()
  {
    using namespace netstream;
    try
    {
      m_listen_socket = create_tcp_socket();
      if(!m_listen_socket.is_valid())
        return;

      set_reuse_addr(m_listen_socket.native_handle());
      if(!bind_and_listen(m_listen_socket.native_handle(), inputs.port))
        return;

      while(m_running.load(std::memory_order_acquire))
      {
        // Non-blocking accept so we can keep checking m_running.
        set_non_blocking(m_listen_socket.native_handle());

        Socket client;
        while(m_running.load(std::memory_order_acquire))
        {
          client = accept_connection(m_listen_socket.native_handle());
          if(client.is_valid())
            break;
          std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
        if(!client.is_valid())
          break;

        set_blocking(client.native_handle());
        optimize_for_low_latency(client.native_handle());

        m_client_socket = std::move(client);
        m_connected.store(true, std::memory_order_release);

        while(m_running.load(std::memory_order_acquire) && m_client_socket.is_valid())
        {
          if(!receive_frame())
            break;
          refresh_quickack(m_client_socket.native_handle());
        }

        m_client_socket.close();
        m_connected.store(false, std::memory_order_release);
      }
    }
    catch(...)
    {
      // Swallow: a dead I/O thread simply means no new frames arrive.
    }
    m_running.store(false, std::memory_order_release);
  }

  // Read one length-prefixed XYZ-f32 frame straight into the write buffer.
  bool receive_frame()
  {
    using namespace netstream;

    // 4-byte little-endian payload length (in bytes).
    uint32_t payload_bytes = 0;
    std::span<uint8_t> len_span(
        reinterpret_cast<uint8_t*>(&payload_bytes), sizeof(payload_bytes));
    if(!recv_all(m_client_socket.native_handle(), len_span))
      return false;

    // Reject empty, absurdly large, or non-float3-aligned payloads.
    constexpr uint32_t point_bytes = 3 * sizeof(float); // 12
    if(payload_bytes == 0 || payload_bytes >= INT32_MAX
       || (payload_bytes % point_bytes) != 0)
      return false;

    const std::size_t num_floats = payload_bytes / sizeof(float);
    auto& write_buf = m_buffer.write_buffer();
    write_buf.resize(num_floats);

    std::span<uint8_t> dst(
        reinterpret_cast<uint8_t*>(write_buf.data()), payload_bytes);
    if(!recv_all(m_client_socket.native_handle(), dst))
      return false;

    m_buffer.publish();
    return true;
  }

  std::atomic<bool> m_running{false};
  std::atomic<bool> m_connected{false};
  std::thread m_thread;
  netstream::Socket m_listen_socket;
  netstream::Socket m_client_socket;
  netstream::TripleBuffer<float> m_buffer;
};
}
