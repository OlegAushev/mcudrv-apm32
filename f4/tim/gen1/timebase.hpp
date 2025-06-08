#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/tim/gen1/base.hpp>

namespace mcu {
namespace apm32 {
namespace tim {
namespace gen1 {

struct TimebaseConfig {
  float freq;
  bool arr_preload;
  TMR_BaseConfig_T hal_base_config;
};

class TimebaseTimer : public internal::AbstractTimer {
private:
  float freq_{0};
public:
  TimebaseTimer(Peripheral peripheral, TimebaseConfig const& conf);

  static TimebaseTimer* instance(Peripheral peripheral) {
    assert(internal::AbstractTimer::instance(std::to_underlying(peripheral))
               ->mode() == OpMode::timebase);
    return static_cast<TimebaseTimer*>(
        internal::AbstractTimer::instance(std::to_underlying(peripheral)));
  }

  float freq() const { return freq_; }

  void init_interrupts(IrqPriority priority) {
    regs_->DIEN_B.UIEN = 1;
    set_irq_priority(internal::irq_nums[std::to_underlying(peripheral_)],
                     priority);
  }

  void enable_interrupts() {
    regs_->STS_B.UIFLG = 0;
    clear_pending_irq(internal::irq_nums[std::to_underlying(peripheral_)]);
    enable_irq(internal::irq_nums[std::to_underlying(peripheral_)]);
  }

  void disable_interrupts() {
    disable_irq(internal::irq_nums[std::to_underlying(peripheral_)]);
  }

  void ack_interrupt() { regs_->STS_B.UIFLG = 0; }
};

} // namespace gen1
} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
