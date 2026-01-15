#include <apm32/f4/adc/adc_init.hpp>
#include <apm32/f4/adc/adc_utils.hpp>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {
namespace detail {

void init_common() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  initialized = true;

  ADC_CommonConfig_T config{};
  config.mode = ADC_MODE_INDEPENDENT;
  config.prescaler = detail::to_sdk(apm32::f4::adc::v2::calculate_prescaler());
  config.accessMode = ADC_ACCESS_MODE_DISABLED;
  config.twoSampling = ADC_TWO_SAMPLING_5CYCLES;

  ADC_CommonConfig(&config);
}

} // namespace detail
} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
