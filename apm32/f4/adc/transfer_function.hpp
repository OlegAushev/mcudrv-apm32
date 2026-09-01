#pragma once

#include <apm32/f4/adc/low_layer/adc_instances.hpp>

#include <emb/units.hpp>

#include <algorithm>
#include <cstdint>

namespace apm32::f4::adc {

// ADC static transfer function: input voltage <-> output code.
struct transfer_function {
  static constexpr std::uint16_t forward(emb::units::volt_f32 v) {
    return static_cast<std::uint16_t>(
        std::clamp(v.value() * codes_per_volt, 0.f, max_code<float>)
    );
  }

  static constexpr emb::units::volt_f32 inverse(std::uint16_t code) {
    return emb::units::volt_f32{static_cast<float>(code) * lsb};
  }
};

} // namespace apm32::f4::adc
