#include <apm32/f4/adc/adc_init.hpp>
#include <apm32/f4/adc/adc_utils.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace adc {
namespace detail {

void init_common() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  initialized = true;

  auto prescaler_field = detail::prescaler_to_field(
      apm32::f4::adc::calculate_prescaler()
  );

  emb::mmio::modify(ADC123_COMMON->CCTRL,
      emb::mmio::bits<ADC_CCTRL_ADCMSEL>(0u),     // independent mode
      emb::mmio::bits<ADC_CCTRL_ADCPRE>(prescaler_field),
      emb::mmio::bits<ADC_CCTRL_DMAMODEDISSEL>(0u),
      emb::mmio::bits<ADC_CCTRL_SMPDEL2>(0u)      // 5 cycles delay
  );
}

} // namespace detail
} // namespace adc
} // namespace f4
} // namespace apm32
