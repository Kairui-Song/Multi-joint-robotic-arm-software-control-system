#include "arm_control/can_driver.hpp"
#include <zlgcan.h> // Actual vendor header is mandatory; no guessed ABI/dlopen structs.
#include <map>
#include <mutex>

namespace arm_control::can {
struct Device {
  DEVICE_HANDLE handle;
  explicit Device(const CanConfig & c):handle(ZCAN_OpenDevice(c.device_type,c.device_index,0)) {
    if(handle==INVALID_DEVICE_HANDLE) throw std::runtime_error("ZCAN_OpenDevice failed");
  }
  ~Device() {ZCAN_CloseDevice(handle);}
};
static std::mutex devices_mutex;
static std::map<std::pair<unsigned,unsigned>,std::weak_ptr<Device>> devices;
class ZcanDriver final : public Driver {
  std::shared_ptr<Device> device_;
  CHANNEL_HANDLE channel_{INVALID_CHANNEL_HANDLE};
  std::array<ZCAN_Transmit_Data, 256> tx_{};
  std::array<ZCAN_Receive_Data, 256> rx_{};
public:
  ~ZcanDriver() override { close(); }
  void open(const CanConfig & c, const std::vector<Axis> &) override {
    close();
    if (!c.device_type || c.timing0 > 255 || c.timing1 > 255)
      throw std::invalid_argument("Invalid ZCAN device type/timing");
    {
      std::lock_guard<std::mutex> lock(devices_mutex);
      auto & weak=devices[{c.device_type,c.device_index}]; device_=weak.lock();
      if(!device_) {device_=std::make_shared<Device>(c); weak=device_;}
    }
    try {
      if (!c.bitrate_property.empty() || c.fd_adapter) {
        auto * property = GetIProperty(device_->handle);
        if (!property) throw std::runtime_error("ZCAN GetIProperty failed");
        const auto bitrate = std::to_string(c.bitrate);
        const auto data_bitrate=std::to_string(c.data_bitrate), prefix=std::to_string(c.channel)+"/";
        bool okay=false;
        if(property->SetValue) {
          if(c.fd_adapter) okay=property->SetValue((prefix+"canfd_abit_baud_rate").c_str(),bitrate.c_str())==1 &&
            property->SetValue((prefix+"canfd_dbit_baud_rate").c_str(),data_bitrate.c_str())==1;
          else okay=property->SetValue(c.bitrate_property.c_str(),bitrate.c_str())==1;
        }
        ReleaseIProperty(property);
        if (!okay) throw std::runtime_error("ZCAN bitrate property rejected");
      }
      ZCAN_CHANNEL_INIT_CONFIG init{}; init.can_type = c.fd_adapter ? TYPE_CANFD : TYPE_CAN;
      if(c.fd_adapter) {
        init.canfd.acc_code=0; init.canfd.acc_mask=0xffffffff; init.canfd.filter=0; init.canfd.mode=0;
      } else {
        init.can.acc_code = 0; init.can.acc_mask = 0xffffffff;
        init.can.filter = 0; init.can.mode = 0;
        init.can.timing0 = static_cast<unsigned char>(c.timing0);
        init.can.timing1 = static_cast<unsigned char>(c.timing1);
      }
      channel_ = ZCAN_InitCAN(device_->handle, c.channel, &init);
      if (channel_ == INVALID_CHANNEL_HANDLE || ZCAN_ClearBuffer(channel_) != 1 || ZCAN_StartCAN(channel_) != 1)
        throw std::runtime_error("ZCAN channel initialization/start failed");
    } catch (...) { close(); throw; }
  }
  bool transmit(const Frame * frames, size_t count) override {
    if (channel_ == INVALID_CHANNEL_HANDLE || count > tx_.size()) return false;
    for (size_t i = 0; i < count; ++i) {
      if (frames[i].size > 8 || frames[i].id > 0x7ff) return false;
      tx_[i] = {}; tx_[i].frame.can_id = frames[i].id; tx_[i].frame.can_dlc = frames[i].size;
      tx_[i].transmit_type = 0;
      std::copy_n(frames[i].data.data(), frames[i].size, tx_[i].frame.data);
    }
    return ZCAN_Transmit(channel_, tx_.data(), static_cast<unsigned>(count)) == count;
  }
  int receive(Frame * frames, size_t capacity) override {
    if (channel_ == INVALID_CHANNEL_HANDLE) return -1;
    const auto available = ZCAN_GetReceiveNum(channel_, TYPE_CAN);
    if (available > capacity || available > rx_.size()) return -1;
    if (!available) return 0;
    const auto n = ZCAN_Receive(channel_, rx_.data(), available, 0);
    if (n > available) return -1;
    for (size_t i = 0; i < n; ++i) {
      auto & f = frames[i]; const auto & raw = rx_[i].frame;
      if (raw.can_dlc > 8) return -1;
      f = {}; f.id = raw.can_id & 0x1fffffff; f.size = raw.can_dlc;
      f.extended = raw.can_id & 0x80000000; f.remote = raw.can_id & 0x40000000; f.error = raw.can_id & 0x20000000;
      std::copy_n(raw.data, f.size, f.data.data());
    }
    return static_cast<int>(n);
  }
  void close() noexcept override {
    if (channel_ != INVALID_CHANNEL_HANDLE) ZCAN_ResetCAN(channel_);
    channel_ = INVALID_CHANNEL_HANDLE; device_.reset();
  }
};
std::unique_ptr<Driver> make_zcan_driver() { return std::make_unique<ZcanDriver>(); }
} // namespace arm_control::can
