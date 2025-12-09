#pragma once

#include <apm32/f4/adc.hpp>

namespace apm32 {
namespace f4 {
namespace util {

class temperature_sensor {
public:
  static float convert_to_temperature(uint32_t adc_data) {
    float const volt = static_cast<float>(adc_data) * apm32::f4::adc::vref /
                       apm32::f4::adc::nmax<float>;
    return (volt - 0.7782f) / 0.0024f + 28.0f;
  }
};

} // namespace util
} // namespace f4
} // namespace apm32
