#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/gpio.hpp>

namespace apm32 {
namespace f4 {
namespace can {

enum class error : uint8_t {
  timeout,
  overflow,
  internal
};

struct rx_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct tx_pin_config {
  gpio::port port;
  gpio::pin pin;
};

enum class mode : uint32_t {
  normal          = 0b00,
  loopback        = 0b01,
  silent          = 0b10,
  silent_loopback = 0b11,
};

enum class rx_fifo : uint32_t { _0, _1 };

} // namespace can
} // namespace f4
} // namespace apm32
