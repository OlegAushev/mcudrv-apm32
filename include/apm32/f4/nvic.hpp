#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/core.hpp>

#include <emb/math.hpp>

#include <algorithm>

namespace apm32 {
namespace f4 {
namespace nvic {

using irq_number = IRQn_Type;

class irq_priority {
  static constexpr uint8_t group_max{15};
  static constexpr uint8_t sub_max{0};
  uint8_t group_;
  uint8_t sub_;
public:
  explicit constexpr irq_priority(uint8_t group, uint8_t sub = 0)
      : group_{std::clamp(group, uint8_t{0}, group_max)},
        sub_{std::clamp(sub, uint8_t{0}, sub_max)} {
    core::ensure(group <= group_max);
    core::ensure(sub <= sub_max);
#ifdef SILICON_REVISION_A
    core::ensure(emb::iseven(group));
#endif
  }

  constexpr uint8_t group() const {
    return group_;
  }

  constexpr uint8_t sub() const {
    return sub_;
  }
};

inline void set_irq_priority(irq_number irqn, irq_priority priority) {
  uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(
      irqn,
      NVIC_EncodePriority(prioritygroup, priority.group(), priority.sub())
  );
}

inline void enable_irq(irq_number irqn) {
  NVIC_EnableIRQ(irqn);
}

inline void disable_irq(irq_number irqn) {
  NVIC_DisableIRQ(irqn);
}

inline void clear_pending_irq(irq_number irqn) {
  NVIC_ClearPendingIRQ(irqn);
}

inline void enable_interrupts() {
  __enable_irq();
}

inline void disable_interrupts() {
  __disable_irq();
}

class irq_guard {
public:
  irq_guard() : primask_backup_(__get_PRIMASK()) {
    __disable_irq();
  }

  irq_guard(const irq_guard&) = delete;
  irq_guard& operator=(const irq_guard&) = delete;

  ~irq_guard() {
    unlock();
  }

  void unlock() {
    if (locked_) {
      locked_ = false;
      if (primask_backup_ == 0) {
        __enable_irq();
      }
    }
  }
private:
  uint32_t primask_backup_;
  bool locked_ = true;
};

} // namespace nvic
} // namespace f4
} // namespace apm32
