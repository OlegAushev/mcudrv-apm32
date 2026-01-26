#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>
#include <apm32/f4/tim/timer_utils.hpp>

#include <apm32f4xx_tmr.h>

namespace apm32 {
namespace f4 {
namespace tim {
namespace hall {

template<timer_instance Tim>
  requires(
      std::same_as<typename Tim::counter_type, uint32_t> &&
      Tim::io_channel_count >= 3
  )
class hall_interface {
public:
  using timer_instance = Tim;
private:
  static inline registers& regs_ = timer_instance::regs;
  static inline nvic::irq_number const cc_irqn_ =
      timer_instance::capture_compare_irqn;
};

}
} // namespace tim
} // namespace f4
} // namespace apm32
