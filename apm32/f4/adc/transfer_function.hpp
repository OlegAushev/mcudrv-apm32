#pragma once

#include <apm32/f4/adc/low_layer/adc_instances.hpp>

#include <cstdint>

namespace apm32::f4::adc {

// ADC static transfer function: input voltage <-> output code.
struct transfer_function {
  static constexpr std::uint16_t forward(float v) {
    return static_cast<std::uint16_t>(v * nmax<float> / vref);
  }

  static constexpr float inverse(std::uint16_t code) {
    return static_cast<float>(code) * vref / nmax<float>;
  }
};

} // namespace apm32::f4::adc
