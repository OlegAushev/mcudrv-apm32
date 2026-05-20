#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/gpio.hpp>

#include <emb/can.hpp>

namespace apm32 {
namespace f4 {
namespace can {

enum class error : uint8_t { timeout, overflow, internal };

struct rx_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct tx_pin_config {
  gpio::port port;
  gpio::pin pin;
};

enum class mode : uint32_t {
  normal = 0b00,
  loopback = 0b01,
  silent = 0b10,
  silent_loopback = 0b11,
};

enum class rx_fifo : uint32_t { _0, _1 };

enum class filter_scale : uint32_t { _16bit, _32bit };

enum class filter_mode : uint32_t { mask, list };

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
  uint16_t id1;
  uint16_t mask1;
  uint16_t id2;
  uint16_t mask2;
};

struct filter_16_list {
  uint16_t id1;
  uint16_t id2;
  uint16_t id3;
  uint16_t id4;
};

} // namespace can
} // namespace f4
} // namespace apm32
