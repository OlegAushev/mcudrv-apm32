#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx.h>

#include <mcudrv-apm32/f4/apm32f4.hpp>

#include <algorithm>
#include <chrono>

namespace mcu {
namespace apm32 {

void init_clk();

void update_clk();

inline uint32_t core_clk_freq() {
  return SystemCoreClock;
}

struct CoreConfig {
  NVIC_PRIORITY_GROUP_T prigroup;
};

void init_core(CoreConfig const& config);

void reset_device();

class IrqPriority {
private:
#ifdef SILICON_REVISION_A
  static inline uint8_t preempt_priorities_{8};
#else
  static inline uint8_t preempt_priorities_{16};
#endif
  static inline uint8_t sub_priorities_{0};

  uint8_t preempt_pri_;
  uint8_t sub_pri_;
public:
  explicit IrqPriority(uint8_t preempt_pri, uint8_t sub_pri)
      : preempt_pri_{std::clamp(preempt_pri, uint8_t{0}, preempt_priorities_)},
        sub_pri_{std::clamp(sub_pri, uint8_t{0}, sub_priorities_)} {
    assert(preempt_pri <= preempt_priorities_);
    assert(sub_pri <= sub_priorities_);
  }

  uint8_t preempt_pri() const { return preempt_pri_; }

  uint8_t sub_pri() const { return sub_pri_; }
};

inline void set_irq_priority(IRQn_Type irqn, IrqPriority priority) {
  uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  NVIC_EnableIRQRequest(irqn,
                        priority.preempt_pri(),
                        priority.sub_pri()); // FIXME remove after testing
  NVIC_SetPriority(irqn,
                   NVIC_EncodePriority(prioritygroup,
                                       priority.preempt_pri(),
                                       priority.sub_pri()));
}

inline void enable_irq(IRQn_Type irqn) {
  NVIC_EnableIRQ(irqn);
}

inline void disable_irq(IRQn_Type irqn) {
  NVIC_DisableIRQ(irqn);
}

inline void clear_pending_irq(IRQn_Type irqn) {
  NVIC_ClearPendingIRQ(irqn);
}

inline void enable_interrupts() {
  __enable_irq();
}

inline void disable_interrupts() {
  __disable_irq();
}

class critical_section {
private:
  bool const irq_enabled;
public:
  bool a() const { return irq_enabled; }

  bool b() const { return irq_enabled; }

  critical_section() : irq_enabled{__get_PRIMASK() == 0} { __disable_irq(); }

  ~critical_section() {
    if (irq_enabled) {
      __enable_irq();
    }
  }
};

} // namespace apm32
} // namespace mcu

#endif
#endif
