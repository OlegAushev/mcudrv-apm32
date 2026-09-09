#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/gpio/gpio.hpp>

#include <emb/assert.hpp>
#include <emb/can.hpp>

#include <cstdint>

namespace apm32::f4::can {

enum class error { timeout, overflow, unknown };

struct rx_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct tx_pin_config {
  gpio::port port;
  gpio::pin pin;
};

enum class mode : std::uint32_t {
  normal = 0b00,
  loopback = 0b01,
  silent = 0b10,
  silent_loopback = 0b11,
};

struct bit_timing {
  static constexpr std::uint32_t prescaler_max = 1024;
  static constexpr std::uint32_t sync_jump_width_max = 4;
  static constexpr std::uint32_t time_segment1_max = 16;
  static constexpr std::uint32_t time_segment2_max = 8;

  std::uint16_t prescaler;
  std::uint8_t sync_jump_width;
  std::uint8_t time_segment1;
  std::uint8_t time_segment2;

  bool operator==(bit_timing const&) const = default;

  constexpr std::uint32_t reg_bits() const
  {
    emb::ensure(prescaler >= 1 && prescaler <= prescaler_max);
    emb::ensure(sync_jump_width >= 1 && sync_jump_width <= sync_jump_width_max);
    emb::ensure(time_segment1 >= 1 && time_segment1 <= time_segment1_max);
    emb::ensure(time_segment2 >= 1 && time_segment2 <= time_segment2_max);

    return ((prescaler - 1u) << CAN_BITTIM_BRPSC_Pos)
         | ((sync_jump_width - 1u) << CAN_BITTIM_RSYNJW_Pos)
         | ((time_segment1 - 1u) << CAN_BITTIM_TIMSEG1_Pos)
         | ((time_segment2 - 1u) << CAN_BITTIM_TIMSEG2_Pos);
  }
};

enum class rx_fifo : std::uint32_t { _0, _1 };

enum class filter_scale : std::uint32_t { _16bit, _32bit };

enum class filter_mode : std::uint32_t { mask, list };

struct filter_32_mask {
  emb::can::format_t format;
  emb::can::id_t id;
  emb::can::id_t mask;
};

struct filter_32_list {
  emb::can::format_t format;
  emb::can::id_t id1;
  emb::can::id_t id2;
};

struct filter_16_mask {
  std::uint16_t id1;
  std::uint16_t mask1;
  std::uint16_t id2;
  std::uint16_t mask2;
};

struct filter_16_list {
  std::uint16_t id1;
  std::uint16_t id2;
  std::uint16_t id3;
  std::uint16_t id4;
};

} // namespace apm32::f4::can
