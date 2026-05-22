#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/gpio/gpio.hpp>

namespace apm32::f4::gpio {

class analog_pin : public detail::pin_base {
public:
  explicit analog_pin(analog_pin_config const& conf) : detail::pin_base(conf) {}

  analog_pin(analog_pin const& other) = delete;
  analog_pin& operator=(analog_pin const& other) = delete;
  analog_pin(analog_pin&& other) = delete;
  analog_pin& operator=(analog_pin&& other) = delete;
};

} // namespace apm32::f4::gpio
