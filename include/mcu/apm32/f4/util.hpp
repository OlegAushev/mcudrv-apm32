#pragma once

#include <mcu/apm32/f4/adc.hpp>

#include <emb/filter.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace util {

class temperature_sensor {
public:
  static float convert_to_temperature(uint32_t adc_data) {
    float volt{
        static_cast<float>(adc_data) * mcu::apm32::adc::vref() /
        mcu::apm32::adc::nmax<float>()};
    return (volt - 0.7782f) / 0.0024f + 28.0f;
  }
};

} // namespace util
} // namespace f4
} // namespace apm32
} // namespace mcu
