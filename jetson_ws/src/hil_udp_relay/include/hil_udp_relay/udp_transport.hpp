// udp_transport.hpp — non-blocking UDP send/recv + fragmentation + reassembly.
//
// WHY THIS EXISTS
//   Companion to udp_protocol.hpp. Provides the actual socket plumbing shared by
//   every relay node: a UdpSender that fragments a serialized payload across
//   datagrams, and a UdpReceiver that reassembles them back into whole messages.
//   ROS-distro-agnostic (POSIX sockets only) so the file is copied verbatim into
//   both the ROS 1 and ROS 2 packages — keep the two copies identical.
//
// GOTCHAS
//   * Sockets are non-blocking. The receiver is meant to be driven from its own
//     recv thread (poll() with a short timeout) so the ROS spin thread is never
//     stalled on socket I/O.
//   * SO_SNDBUF / SO_RCVBUF are bumped to ~4 MB so a burst of ~270 KB CustomMsg
//     fragments isn't dropped by an undersized kernel buffer. The kernel may cap
//     below the request (net.core.rmem_max); we log the achieved size.
//   * Reassembly keys fragments by seq. A partial set is evicted when a newer
//     seq for the same msg_type arrives or when it exceeds a staleness timeout.
//     UDP gives no delivery/ordering guarantee — a whole message is simply lost
//     if any fragment drops; that is acceptable for sensor/cmd streaming.

#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "hil_udp_relay/udp_protocol.hpp"

namespace hil_udp_relay {

inline uint32_t now_ms() {
  using namespace std::chrono;
  return static_cast<uint32_t>(
      duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

// ---------------------------------------------------------------------------
// UdpSender — send a full serialized payload, fragmenting as needed.
// ---------------------------------------------------------------------------
class UdpSender {
 public:
  UdpSender() = default;
  ~UdpSender() {
    if (fd_ >= 0) ::close(fd_);
  }

  // dst_ip:dst_port is the remote endpoint datagrams are sent to.
  bool open(const std::string& dst_ip, uint16_t dst_port, int sndbuf = 4 << 20) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    ::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    std::memset(&dst_, 0, sizeof(dst_));
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(dst_port);
    if (::inet_pton(AF_INET, dst_ip.c_str(), &dst_.sin_addr) != 1) return false;
    return true;
  }

  // Serialize already done by caller into `payload`; we add headers + fragment.
  void send(uint16_t msg_type, const std::vector<uint8_t>& payload) {
    if (fd_ < 0) return;
    const uint32_t total = static_cast<uint32_t>(payload.size());
    const uint32_t frag_count =
        total == 0 ? 1 : (total + kMaxFragPayload - 1) / kMaxFragPayload;
    const uint32_t seq = seq_++;
    std::vector<uint8_t> dgram;
    dgram.reserve(kHeaderSize + kMaxFragPayload);
    for (uint32_t fi = 0; fi < frag_count; ++fi) {
      const uint32_t off = fi * kMaxFragPayload;
      const uint32_t len = std::min(kMaxFragPayload, total - off);
      PacketHeader h;
      h.magic = kMagic;
      h.msg_type = msg_type;
      h.flags = 0;
      h.seq = seq;
      h.total_len = total;
      h.frag_offset = off;
      h.frag_index = static_cast<uint16_t>(fi);
      h.frag_count = static_cast<uint16_t>(frag_count);
      dgram.clear();
      const auto* hp = reinterpret_cast<const uint8_t*>(&h);
      dgram.insert(dgram.end(), hp, hp + kHeaderSize);
      if (len > 0)
        dgram.insert(dgram.end(), payload.begin() + off, payload.begin() + off + len);
      // Non-blocking send; on EWOULDBLOCK the datagram is simply dropped (the
      // next message supersedes it for streaming topics).
      ::sendto(fd_, dgram.data(), dgram.size(), 0,
               reinterpret_cast<sockaddr*>(&dst_), sizeof(dst_));
    }
  }

  bool valid() const { return fd_ >= 0; }

 private:
  int fd_ = -1;
  sockaddr_in dst_{};
  uint32_t seq_ = 0;
};

// ---------------------------------------------------------------------------
// UdpReceiver — bind a port, recv datagrams, reassemble, invoke callback with
// the complete payload + msg_type. Run poll_once() from a recv thread loop.
// ---------------------------------------------------------------------------
class UdpReceiver {
 public:
  // Callback receives (msg_type, full reassembled payload).
  using Callback = std::function<void(uint16_t, const std::vector<uint8_t>&)>;

  UdpReceiver() = default;
  ~UdpReceiver() {
    if (fd_ >= 0) ::close(fd_);
  }

  bool open(uint16_t bind_port, Callback cb, int rcvbuf = 4 << 20) {
    cb_ = std::move(cb);
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    int reuse = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(bind_port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  // Drain all currently-available datagrams; `timeout_ms` is how long poll()
  // waits when the socket is idle. Returns false if the socket is invalid.
  bool poll_once(int timeout_ms = 50) {
    if (fd_ < 0) return false;
    pollfd pfd{fd_, POLLIN, 0};
    int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr <= 0) {
      evict_stale();
      return true;
    }
    std::vector<uint8_t> buf(kHeaderSize + kMaxFragPayload + 64);
    for (;;) {
      ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
      if (n < 0) break;  // EWOULDBLOCK: drained
      if (static_cast<size_t>(n) < kHeaderSize) continue;
      handle_datagram(buf.data(), static_cast<size_t>(n));
    }
    evict_stale();
    return true;
  }

  bool valid() const { return fd_ >= 0; }

 private:
  struct Pending {
    uint16_t msg_type = 0;
    uint32_t total_len = 0;
    uint16_t frag_count = 0;
    uint16_t got = 0;
    uint32_t last_ms = 0;
    std::vector<uint8_t> buf;        // total_len bytes, filled by offset
    std::vector<bool> have;          // per-fragment received flag
  };

  void handle_datagram(const uint8_t* data, size_t n) {
    PacketHeader h;
    std::memcpy(&h, data, kHeaderSize);
    if (h.magic != kMagic) return;
    const uint8_t* frag = data + kHeaderSize;
    const size_t frag_len = n - kHeaderSize;

    // Fast path: single-fragment message — deliver immediately.
    if (h.frag_count <= 1) {
      std::vector<uint8_t> payload(frag, frag + frag_len);
      if (cb_) cb_(h.msg_type, payload);
      return;
    }

    // Multi-fragment: reassemble keyed by seq.
    auto& p = pending_[h.seq];
    if (p.frag_count == 0) {  // first fragment of this seq
      p.msg_type = h.msg_type;
      p.total_len = h.total_len;
      p.frag_count = h.frag_count;
      p.buf.assign(h.total_len, 0);
      p.have.assign(h.frag_count, false);
      // A newer seq supersedes any older incomplete sets of the same type.
      drop_older_than(h.msg_type, h.seq);
    }
    p.last_ms = now_ms();
    if (h.frag_index < p.frag_count && !p.have[h.frag_index]) {
      if (static_cast<size_t>(h.frag_offset) + frag_len <= p.buf.size()) {
        std::memcpy(p.buf.data() + h.frag_offset, frag, frag_len);
        p.have[h.frag_index] = true;
        ++p.got;
      }
    }
    if (p.got == p.frag_count) {
      std::vector<uint8_t> payload = std::move(p.buf);
      uint16_t mt = p.msg_type;
      pending_.erase(h.seq);
      if (cb_) cb_(mt, payload);
    }
  }

  // Remove incomplete sets of `mt` whose seq is older than `seq` (wrap-safe via
  // signed diff over uint32).
  void drop_older_than(uint16_t mt, uint32_t seq) {
    for (auto it = pending_.begin(); it != pending_.end();) {
      const bool older =
          static_cast<int32_t>(seq - it->first) > 0 && it->second.msg_type == mt;
      if (older)
        it = pending_.erase(it);
      else
        ++it;
    }
  }

  void evict_stale() {
    const uint32_t t = now_ms();
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (t - it->second.last_ms > kStaleMs)
        it = pending_.erase(it);
      else
        ++it;
    }
  }

  static constexpr uint32_t kStaleMs = 500;  // drop partial sets after 0.5 s

  int fd_ = -1;
  Callback cb_;
  std::map<uint32_t, Pending> pending_;  // keyed by seq
};

}  // namespace hil_udp_relay
