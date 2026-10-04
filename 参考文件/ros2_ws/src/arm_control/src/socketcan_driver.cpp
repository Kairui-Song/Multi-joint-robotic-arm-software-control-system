#include "arm_control/can_driver.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace arm_control::can {
class SocketDriver final : public Driver {
  int fd_{-1};
public:
  ~SocketDriver() override { close(); }
  void open(const CanConfig & config, const std::vector<Axis> & axes) override {
    close(); fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("Cannot create SocketCAN socket");
    try {
      std::vector<can_filter> filters;
      for (const auto & axis : axes) filters.push_back({
        static_cast<canid_t>(config.protocol=="mit_shared_v1" ? 0 : axis.can.motor_id+(config.protocol=="robomaster_c620"?0x200:0)), CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG});
      can_err_mask_t errors = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_TX_TIMEOUT;
      int receive_own = 0;
      if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(can_filter)) ||
          setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) ||
          setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own, sizeof(receive_own)))
        throw std::runtime_error("SocketCAN filter configuration failed");
      sockaddr_can address{}; address.can_family = AF_CAN;
      address.can_ifindex = static_cast<int>(if_nametoindex(config.interface.c_str()));
      if (!address.can_ifindex || bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)))
        throw std::runtime_error("Cannot bind CAN interface; configure bitrate and bring interface up first");
    } catch (...) { close(); throw; }
  }
  bool transmit(const Frame * frames, size_t count) override {
    for (size_t i = 0; i < count; ++i) {
      can_frame raw{}; raw.can_id = frames[i].id; raw.can_dlc = frames[i].size;
      if (raw.can_dlc > 8 || raw.can_id > CAN_SFF_MASK) return false;
      std::copy_n(frames[i].data.data(), raw.can_dlc, raw.data);
      if (::write(fd_, &raw, sizeof(raw)) != sizeof(raw)) return false;
    }
    return true;
  }
  int receive(Frame * frames, size_t capacity) override {
    size_t n = 0;
    while (n <= capacity) {
      can_frame raw{}; const auto bytes = ::read(fd_, &raw, sizeof(raw));
      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return static_cast<int>(n);
      if (bytes != sizeof(raw) || n == capacity || raw.can_dlc > 8) return -1;
      auto & f = frames[n++]; f = {}; f.id = raw.can_id & CAN_EFF_MASK; f.size = raw.can_dlc;
      f.extended = raw.can_id & CAN_EFF_FLAG; f.remote = raw.can_id & CAN_RTR_FLAG; f.error = raw.can_id & CAN_ERR_FLAG;
      std::copy_n(raw.data, f.size, f.data.data());
    }
    return -1;
  }
  void close() noexcept override { if (fd_ >= 0) ::close(fd_); fd_ = -1; }
};
std::unique_ptr<Driver> make_socketcan_driver() { return std::make_unique<SocketDriver>(); }
} // namespace arm_control::can
