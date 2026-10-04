#pragma once
#include "arm_control/can_protocol.hpp"
#include "arm_control/core.hpp"

// Exact profile from quanbu_tongshi.py: standard ID 0, payload master ID.
// Deliberately not generic Damiao: other firmware may encode ID/status differently.
namespace arm_control::mit {
inline can::Frame mode(uint16_t id, bool enable) {
  can::check_id(id);
  can::Frame f; f.id=id; f.size=8; f.data.fill(0xff);
  f.data[7]=enable ? 0xfc : 0xfd; return f;
}
inline can::Frame command(const CanAxis & a, double position, bool powered=true) {
  can::check_id(a.motor_id);
  const auto p=can::pack(position,-a.p_max,a.p_max,16);
  const auto v=can::pack(0,-a.v_max,a.v_max,12);
  const auto t=can::pack(0,-a.t_max,a.t_max,12);
  const auto k=can::pack(powered?a.kp:0,0,500,12);
  const auto d=can::pack(powered?a.kd:0,0,5,12);
  can::Frame f; f.id=a.motor_id; f.size=8;
  f.data={uint8_t(p>>8),uint8_t(p),uint8_t(v>>4),uint8_t((v<<4)|(k>>8)),
    uint8_t(k),uint8_t(d>>4),uint8_t((d<<4)|(t>>8)),uint8_t(t)};
  return f;
}
inline bool decode(const can::Frame & f, const CanAxis & a, can::Sample & s) {
  if(f.id!=0 || f.size!=8 || f.extended || f.remote || f.error || f.data[0]!=a.master_id) return false;
  s.position=can::unpack((uint32_t(f.data[1])<<8)|f.data[2],-a.p_max,a.p_max,16);
  s.velocity=can::unpack((uint32_t(f.data[3])<<4)|(f.data[4]>>4),-a.v_max,a.v_max,12);
  s.current=can::unpack((uint32_t(f.data[4]&15)<<8)|f.data[5],-a.t_max,a.t_max,12);
  s.temperature=std::max(f.data[6],f.data[7]); s.error=0; s.ack=0;
  return true;
}
}
