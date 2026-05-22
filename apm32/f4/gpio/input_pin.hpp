#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/gpio/gpio.hpp>

#include <emb/gpio.hpp>

namespace apm32::f4::gpio {

class input_pin : public detail::pin_base {
private:
  emb::gpio::polarity const polarity_;
public:
  explicit input_pin(input_pin_config const& conf)
      : detail::pin_base(conf), polarity_(conf.polarity) {}

  input_pin(input_pin const& other) = delete;
  input_pin& operator=(input_pin const& other) = delete;

  emb::gpio::polarity polarity() const {
    return polarity_;
  }

  emb::gpio::level read_level() const {
    if ((regs_->IDATA & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }

  emb::gpio::state read() const {
    return emb::gpio::to_state(read_level(), polarity_);
  }
};

static_assert(emb::gpio::input<input_pin>);

} // namespace apm32::f4::gpio
