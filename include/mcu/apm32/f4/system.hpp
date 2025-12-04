#pragma once

#ifdef APM32F4XX

#include <apm32f4xx.h>

#include <mcu/apm32/f4/apm32f4.hpp>

#include <algorithm>
#include <chrono>
#include <emb/math.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {

void init_clk();

void update_clk();

inline uint32_t core_clk_freq() {
  return SystemCoreClock;
}

void init_core();

void reset_device();

class IrqPriority {
  static constexpr uint8_t group_max{15};
  static constexpr uint8_t sub_max{0};
  uint8_t group_;
  uint8_t sub_;
public:
  explicit constexpr IrqPriority(uint8_t group, uint8_t sub = 0)
      : group_{std::clamp(group, uint8_t{0}, group_max)},
        sub_{std::clamp(sub, uint8_t{0}, sub_max)} {
    assert(group <= group_max);
    assert(sub <= sub_max);
#ifdef SILICON_REVISION_A
    assert(emb::iseven(group));
#endif
  }

  uint8_t group() const { return group_; }

  uint8_t sub() const { return sub_; }
};

inline void set_irq_priority(IRQn_Type irqn, IrqPriority priority) {
  uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(
      irqn,
      NVIC_EncodePriority(prioritygroup, priority.group(), priority.sub()));
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
  bool enable_irq_on_exit_;
public:
  critical_section() : enable_irq_on_exit_{__get_PRIMASK() == 0} {
    __disable_irq();
  }

  void exit() {
    if (enable_irq_on_exit_) {
      __enable_irq();
      enable_irq_on_exit_ = false;
    }
  }

  ~critical_section() {
    if (enable_irq_on_exit_) {
      __enable_irq();
    }
  }
};

class Mutex {
private:
  bool locked_ = false;
  bool enable_irq_on_unlock = false;
public:
  void lock() {
    assert(!locked_);
    locked_ = true;
    if (__get_PRIMASK() == 0) {
      enable_irq_on_unlock = true;
      __disable_irq();
    }
  }

  void unlock() {
    assert(locked_);
    if (enable_irq_on_unlock) {
      enable_irq_on_unlock = false;
      __enable_irq();
    }
    locked_ = false;
  }
};

inline uint32_t serial_number() {
  uint32_t* uid_ptr{reinterpret_cast<uint32_t*>(0x1FFF7A10)};
  return *uid_ptr;
}

} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
