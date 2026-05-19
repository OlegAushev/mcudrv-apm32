#pragma once

#include <apm32/f4/can/can_instances.hpp>
#include <apm32/f4/can/can_types.hpp>

namespace apm32 {
namespace f4 {
namespace can {

struct filter_init_session {
  filter_init_session() {
    emb::mmio::set(can1::reg.FCTRL, CAN_FCTRL_FINITEN);
  }

  ~filter_init_session() {
    emb::mmio::clear(can1::reg.FCTRL, CAN_FCTRL_FINITEN);
  }
};

inline void setup_filter_bank(
    filter_scale scale,
    filter_mode mode,
    rx_fifo fifo,
    size_t filter_idx,
    uint32_t bank1,
    uint32_t bank2
) {
  registers& reg = can1::reg;

  uint32_t const filter_bit = 1u << filter_idx;

  filter_init_session fg;

  // deactivate filter
  emb::mmio::clear(reg.FACT, filter_bit);

  emb::mmio::set_or_clear(reg.FSCFG, filter_bit, scale == filter_scale::_32bit);
  emb::mmio::set_or_clear(reg.FMCFG, filter_bit, mode == filter_mode::list);
  emb::mmio::set_or_clear(reg.FFASS, filter_bit, fifo == rx_fifo::_1);

  reg.sFilterRegister[filter_idx].FBANK1 = bank1;
  reg.sFilterRegister[filter_idx].FBANK2 = bank2;

  // activate filter
  emb::mmio::set(reg.FACT, filter_bit);
}

namespace detail {

constexpr uint32_t encode_32bit_id(emb::can::format_t fmt, emb::can::id_t id) {
  if (fmt == emb::can::format_t::standard) {
    return (id & 0x7FFu) << 21;
  }
  constexpr uint32_t ide_bit = 1u << 2;
  return (id & 0x1FFFFFFFu) << 3 | ide_bit;
}

constexpr uint32_t
encode_32bit_mask(emb::can::format_t fmt, emb::can::id_t mask) {
  constexpr uint32_t rtr_bit = 1u << 1; // accept data frames only
  constexpr uint32_t ide_bit = 1u << 2;
  if (fmt == emb::can::format_t::standard) {
    return (mask & 0x7FFu) << 21 | ide_bit | rtr_bit;
  }
  return (mask & 0x1FFFFFFFu) << 3 | ide_bit | rtr_bit;
}

constexpr uint32_t encode_16bit_id(emb::can::id_t id) {
  return (id & 0x7FFu) << 5;
}

constexpr uint32_t encode_16bit_mask(emb::can::id_t id) {
  constexpr uint32_t rtr_bit = 1u << 4; // accept data frames only
  constexpr uint32_t ide_bit = 1u << 3;
  return (id & 0x7FFu) << 5 | ide_bit | rtr_bit;
}

} // namespace detail

} // namespace can
} // namespace f4
} // namespace apm32
