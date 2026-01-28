#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>
#include <apm32/f4/tim/timer_utils.hpp>

#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_tmr.h>

#include <optional>

namespace apm32 {
namespace f4 {
namespace tim {
namespace hall {

struct input_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct hall_interface_config {
  nvic::irq_priority cc_irq_priority;
  std::array<input_pin_config, 3> pins;
};

template<timer_instance Tim>
  requires(
      std::same_as<typename Tim::counter_type, uint32_t> &&
      Tim::io_channel_count >= 3
  )
class hall_interface {
public:
  using timer_instance = Tim;
  using reg_addr = timer_instance::reg_addr;
private:
  static inline registers& regs_ = timer_instance::regs;
  static inline nvic::irq_number const cc_irqn_ =
      timer_instance::capture_compare_irqn;

  std::array<std::optional<gpio::alternate_pin>, 3> pins_;
public:
};

}
} // namespace tim
} // namespace f4
} // namespace apm32
