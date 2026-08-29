#pragma once

#include <emb/units.hpp>

namespace apm32::f4::adc {

// Internal temperature sensor: junction temperature <-> sensor output voltage,
// the datasheet's linear model V(T) = v0 + slope * (T - t0).
struct temperature_sensor {
  static constexpr float T0 = 28.f;       // degC
  static constexpr float V0 = 0.7782f;    // V at t0
  static constexpr float slope = 0.0024f; // V/degC

  static constexpr float forward(emb::units::degree_celsius_f32 t) {
    return V0 + slope * (t.value() - T0);
  }

  static constexpr emb::units::degree_celsius_f32 inverse(float v) {
    return emb::units::degree_celsius_f32{(v - V0) / slope + T0};
  }
};

} // namespace apm32::f4::adc
