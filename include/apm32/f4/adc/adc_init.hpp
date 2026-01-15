#pragma once

#include <apm32/f4/adc/adc_types.hpp>
#include <apm32/f4/nvic.hpp>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

inline nvic::irq_priority common_irq_priority{0};

namespace detail {
void init_common();
} // namespace detail

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
