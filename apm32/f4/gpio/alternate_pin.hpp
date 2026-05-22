#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/gpio/gpio.hpp>

#include <emb/gpio.hpp>

namespace apm32::f4::gpio {

class alternate_pin : public detail::pin_base {
public:
  explicit alternate_pin(alternate_pin_config const& conf)
      : detail::pin_base(conf) {}

  alternate_pin(alternate_pin const& other) = delete;
  alternate_pin& operator=(alternate_pin const& other) = delete;

  emb::gpio::level read_level() const {
    if ((regs_->IDATA & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }
};

} // namespace apm32::f4::gpio
