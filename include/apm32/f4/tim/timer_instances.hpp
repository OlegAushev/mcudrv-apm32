#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/tim/timer_registers.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/meta.hpp>
#include <emb/mmio.hpp>

#include <array>
#include <cstddef>
#include <utility>

namespace apm32 {
namespace f4 {
namespace tim {

using registers = TMR_TypeDef;

inline constexpr size_t timer_count = 14;

struct tim1 {
  static constexpr uintptr_t base_addr = TMR1_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR1;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR1_UP_TMR10_IRQn;
  static constexpr nvic::irq_number break_irqn = TMR1_BRK_TMR9_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR1_CC_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb2_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR1EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::tmr1;
};

struct tim2 {
  static constexpr uintptr_t base_addr = TMR2_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR2;

  using counter_type = uint32_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR2_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR2_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR2EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::tmr2;
};

struct tim3 {
  static constexpr uintptr_t base_addr = TMR3_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR3;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR3_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR3_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR3EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::tmr3;
};

struct tim4 {
  static constexpr uintptr_t base_addr = TMR4_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR4;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR4_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR4_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR4EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::tmr4;
};

struct tim5 {
  static constexpr uintptr_t base_addr = TMR5_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR5;

  using counter_type = uint32_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR5_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR5_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR5EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::tmr5;
};

struct tim6 {
  static constexpr uintptr_t base_addr = TMR6_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR6;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 0;

  static void enable_clock() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR6EN);
  }
};

struct tim7 {
  static constexpr uintptr_t base_addr = TMR7_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR7;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 0;

  static void enable_clock() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR7EN);
  }
};

struct tim8 {
  static constexpr uintptr_t base_addr = TMR8_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline registers& regs = *TMR8;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR8_UP_TMR13_IRQn;
  static constexpr nvic::irq_number break_irqn = TMR8_BRK_TMR12_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR8_CC_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb2_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR8EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::tmr8;
};

struct tim9 {
  static inline registers& regs = *TMR9;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 2;

  static void enable_clock() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR9EN);
  }
};

struct tim10 {
  static inline registers& regs = *TMR10;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR10EN);
  }
};

struct tim11 {
  static inline registers& regs = *TMR11;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_TMR11EN);
  }
};

struct tim12 {
  static inline registers& regs = *TMR12;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 2;

  static void enable_clock() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR12EN);
  }
};

struct tim13 {
  static inline registers& regs = *TMR13;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR13EN);
  }
};

struct tim14 {
  static inline registers& regs = *TMR14;

  using counter_type = uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_TMR14EN);
  }
};

template<typename T>
struct is_timer_instance : std::bool_constant<emb::same_as_any<
                               T,
                               tim1,
                               tim2,
                               tim3,
                               tim4,
                               tim5,
                               tim6,
                               tim7,
                               tim8,
                               tim9,
                               tim10,
                               tim11,
                               tim12,
                               tim13,
                               tim14>> {};

template<typename T>
concept timer_instance = is_timer_instance<T>::value;

template<typename T>
struct is_advanced_timer : std::bool_constant<emb::same_as_any<T, tim1, tim8>> {
};

template<typename T>
concept advanced_timer = is_advanced_timer<T>::value;

template<typename T>
struct is_general_purpose_timer : std::bool_constant<emb::same_as_any<
                                      T,
                                      tim2,
                                      tim3,
                                      tim4,
                                      tim5,
                                      tim9,
                                      tim10,
                                      tim11,
                                      tim12,
                                      tim13,
                                      tim14>> {};

template<typename T>
concept general_purpose_timer = is_general_purpose_timer<T>::value;

template<typename T>
struct is_32bit_timer
    : std::bool_constant<std::same_as<typename T::counter_type, uint32_t>> {};

template<typename T>
concept timer_32bit = is_32bit_timer<T>::value;

template<typename T>
struct is_basic_timer : std::bool_constant<emb::same_as_any<T, tim6, tim7>> {};

template<typename T>
concept basic_timer = is_basic_timer<T>::value;

} // namespace tim
} // namespace f4
} // namespace apm32
