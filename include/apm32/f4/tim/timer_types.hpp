#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_rcm.h>
#include <apm32f4xx_tmr.h>

#include <emb/units.hpp>

#include <array>
#include <cstddef>
#include <utility>

namespace apm32 {
namespace f4 {
namespace tim {

using peripheral_registers = TMR_T;

inline constexpr size_t peripheral_count = 14;

struct tim1 {
  static inline peripheral_registers* regs = TMR1;

  using counter_type = uint16_t;

  template<typename T>
  static constexpr auto clock_frequency = core::apb2_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR1);
  };

  static constexpr nvic::irq_number update_irqn = TMR1_UP_TMR10_IRQn;

  static constexpr nvic::irq_number break_irqn = TMR1_BRK_TMR9_IRQn;

  static inline std::array<uint32_t volatile*, 4> ccr_regs =
      {&regs->CC1, &regs->CC2, &regs->CC3, &regs->CC4};
};

struct tim2 {
  static inline peripheral_registers* regs = TMR2;

  using counter_type = uint32_t;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR2);
  };

  static constexpr nvic::irq_number update_irqn = TMR2_IRQn;
};

struct tim3 {
  static inline peripheral_registers* regs = TMR3;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR3);
  }
  using counter_type = uint16_t;
};

struct tim4 {
  static inline peripheral_registers* regs = TMR4;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR4);
  }
  using counter_type = uint16_t;
};

struct tim5 {
  static inline peripheral_registers* regs = TMR5;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR5);
  }
  using counter_type = uint32_t;
};

struct tim6 {
  static inline peripheral_registers* regs = TMR6;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR6);
  }
  using counter_type = uint16_t;
};

struct tim7 {
  static inline peripheral_registers* regs = TMR7;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR7);
  }
  using counter_type = uint16_t;
};

struct tim8 {
  static inline peripheral_registers* regs = TMR8;

  using counter_type = uint16_t;

  template<typename T>
  static constexpr auto clock_frequency = core::apb2_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR8);
  };

  static constexpr nvic::irq_number update_irqn = TMR8_UP_TMR13_IRQn;

  static constexpr nvic::irq_number break_irqn = TMR8_BRK_TMR12_IRQn;

  static inline std::array<uint32_t volatile*, 4> ccr_regs =
      {&regs->CC1, &regs->CC2, &regs->CC3, &regs->CC4};
};

struct tim9 {
  static inline peripheral_registers* regs = TMR9;
  static void enable_clock() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR9);
  }
  using counter_type = uint16_t;
};

struct tim10 {
  static inline peripheral_registers* regs = TMR10;
  static void enable_clock() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR10);
  }
  using counter_type = uint16_t;
};

struct tim11 {
  static inline peripheral_registers* regs = TMR11;
  static void enable_clock() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR11);
  }
  using counter_type = uint16_t;
};

struct tim12 {
  static inline peripheral_registers* regs = TMR12;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR12);
  }
  using counter_type = uint16_t;
};

struct tim13 {
  static inline peripheral_registers* regs = TMR13;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR13);
  }
  using counter_type = uint16_t;
};

struct tim14 {
  static inline peripheral_registers* regs = TMR14;
  static void enable_clock() {
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR14);
  }
  using counter_type = uint16_t;
};

template<typename Tim>
struct is_timer : std::bool_constant<emb::one_of<
                      Tim,
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

template<typename Tim>
concept timer = is_timer<Tim>::value;

template<typename Tim>
struct is_advanced_timer : std::bool_constant<emb::one_of<Tim, tim1, tim8>> {};

template<typename Tim>
concept advanced_timer = is_advanced_timer<Tim>::value;

template<typename Tim>
struct is_general_purpose_timer : std::bool_constant<emb::one_of<
                                      Tim,
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

template<typename Tim>
concept general_purpose_timer = is_general_purpose_timer<Tim>::value;

template<typename Tim>
struct is_32bit_timer : std::bool_constant<emb::one_of<Tim, tim2, tim5>> {};

template<typename Tim>
concept timer_32bit = is_32bit_timer<Tim>::value;

template<typename Tim>
struct is_basic_timer : std::bool_constant<emb::one_of<Tim, tim6, tim7>> {};

template<typename Tim>
concept basic_timer = is_basic_timer<Tim>::value;

template<typename Tim>
struct has_4_channels
    : std::bool_constant<emb::one_of<Tim, tim1, tim2, tim3, tim4, tim5, tim8>> {
};

template<typename Tim>
concept timer_with_4_channels = has_4_channels<Tim>::value;

enum class channel : uint32_t { ch1, ch2, ch3, ch4 };

enum class clock_division : uint32_t { div1, div2, div4 };

enum class count_direction : uint32_t { up, down };

enum class counter_mode : uint32_t { up, down, updown };

namespace detail {

inline TMR_CLOCK_DIV_T to_sdk(clock_division clkdiv) {
  switch (clkdiv) {
  case clock_division::div1:
    return TMR_CLOCK_DIV_1;
  case clock_division::div2:
    return TMR_CLOCK_DIV_2;
  case clock_division::div4:
    return TMR_CLOCK_DIV_4;
  }
  std::unreachable();
}

template<timer Tim>
constexpr uint16_t calculate_prescaler(
    emb::units::hz_f32 clk_freq,
    emb::units::hz_f32 tim_freq,
    counter_mode mode
) {
  uint32_t clk_freq_u32 = static_cast<uint32_t>(clk_freq.numval());
  uint32_t tim_freq_u32 = static_cast<uint32_t>(tim_freq.numval());

  // constexpr replacement for std::div (must be constrexpr since c++23, but...)
  uint32_t total_ticks = clk_freq_u32 / tim_freq_u32 +
                         (clk_freq_u32 % tim_freq_u32 != 0) - 1;
  if (mode == counter_mode::updown) {
    total_ticks = (total_ticks + 1) / 2;
  }

  uint32_t ret = total_ticks /
                 std::numeric_limits<typename Tim::counter_type>::max();
  core::ensure(ret <= UINT16_MAX);

  return static_cast<uint16_t>(ret);
}

} // namespace detail

template<timer Tim>
uint16_t calculate_prescaler(emb::units::hz_f32 tim_freq, counter_mode mode) {
  return detail::calculate_prescaler<Tim>(
      Tim::template clock_frequency<emb::units::hz_f32>(),
      tim_freq,
      mode
  );
}

} // namespace tim
} // namespace f4
} // namespace apm32
