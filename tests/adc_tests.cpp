#include <apm32/f4/adc.hpp>

using namespace apm32::f4::adc::v2;

namespace {

static_assert(detail::to_sdk(2) == ADC_PRESCALER_DIV2);
static_assert(detail::to_sdk(4) == ADC_PRESCALER_DIV4);
static_assert(detail::to_sdk(6) == ADC_PRESCALER_DIV6);
static_assert(detail::to_sdk(8) == ADC_PRESCALER_DIV8);

static_assert(
    detail::calculate_prescaler(
        emb::units::hz_f32{84e6f},
        max_clock_frequency
    ) == 4
);

} // namespace
