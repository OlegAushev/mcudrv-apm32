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
public:
  static constexpr uint8_t preempt_priority_max = 15;
  static constexpr uint8_t sub_priority_max = 0;
private:
  uint8_t preempt_;
  uint8_t sub_;
public:
  explicit constexpr irq_priority(uint8_t preempt_pri, uint8_t sub_pri = 0)
      : preempt_{std::clamp(preempt_pri, uint8_t{0}, preempt_priority_max)},
        sub_{std::clamp(sub_pri, uint8_t{0}, sub_priority_max)} {
    core::ensure(preempt_pri <= preempt_priority_max);
    core::ensure(sub_pri <= sub_priority_max);
#ifdef SILICON_REVISION_A
    core::ensure(emb::iseven(preempt_pri));
#endif
  }

  constexpr uint8_t preempt_priority() const {
    return preempt_;
  }

  constexpr uint8_t sub_priority() const {
    return sub_;
  }
};

inline void set_irq_priority(irq_number irqn, irq_priority priority) {
  uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(
      irqn,
      NVIC_EncodePriority(
          prioritygroup,
          priority.preempt_priority(),
          priority.sub_priority()
      )
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

enum class irq_lock_policy {
  all,     // PRIMASK
  priority // BASEPRI
};

template<
    irq_lock_policy Policy = irq_lock_policy::all,
    uint8_t PreemptThreshold = 0>
  requires(PreemptThreshold <= irq_priority::preempt_priority_max)
class irq_guard {
public:
  irq_guard() {
    if constexpr (Policy == irq_lock_policy::all) {
      backup_ = __get_PRIMASK();
      __disable_irq();
    } else {
      backup_ = __get_BASEPRI();
      __set_BASEPRI(encode_basepri());
    }
  }

  irq_guard(irq_guard const&) = delete;
  irq_guard& operator=(irq_guard const&) = delete;

  ~irq_guard() {
    unlock();
  }

  void unlock() {
    if (locked_) {
      locked_ = false;
      if constexpr (Policy == irq_lock_policy::all) {
        if (backup_ == 0) {
          __enable_irq();
        }
      } else {
        __set_BASEPRI(backup_);
      }
    }
  }
private:
  uint32_t backup_;
  bool locked_ = true;

  static constexpr uint32_t encode_basepri() {
    return PreemptThreshold << (8 - __NVIC_PRIO_BITS);
  }
};

} // namespace nvic
} // namespace f4
} // namespace apm32
