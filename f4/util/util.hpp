#pragma once

#include <mcudrv-apm32/f4/adc/adc.hpp>

#include <emblib/filter.hpp>

namespace mcu {
inline namespace apm32 {
namespace util {

class temperature_sensor {
public:
  static float get_value(uint32_t adc_data) {
    float volt{static_cast<float>(adc_data) * mcu::apm32::adc::vref() /
               mcu::apm32::adc::nmax<float>()};
    return (volt - 0.7782f) / 0.0024f + 28.0f;
  }
};

} // namespace util
} // namespace apm32
} // namespace mcu
