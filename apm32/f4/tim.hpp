#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>
#include <apm32/f4/rcc.hpp>

#include <emb/assert.hpp>
#include <emb/meta.hpp>
#include <emb/mmio.hpp>
#include <emb/units.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace apm32::f4::tim {

using registers = TMR_TypeDef;

inline constexpr std::size_t timer_count = 14;

struct tim1 {
  static inline registers& reg = *TMR1;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR1_UP_TMR10_IRQn;
  static constexpr nvic::irq_number break_irqn = TMR1_BRK_TMR9_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR1_CC_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk2_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB2CLKEN_TMR1EN>(RCM->APB2CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::tmr1;
};

struct tim2 {
  static inline registers& reg = *TMR2;

  using counter_type = std::uint32_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR2_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR2_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB1CLKEN_TMR2EN>(RCM->APB1CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::tmr2;
};

struct tim3 {
  static inline registers& reg = *TMR3;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR3_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR3_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB1CLKEN_TMR3EN>(RCM->APB1CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::tmr3;
};

struct tim4 {
  static inline registers& reg = *TMR4;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR4_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR4_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB1CLKEN_TMR4EN>(RCM->APB1CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::tmr4;
};

struct tim5 {
  static inline registers& reg = *TMR5;

  using counter_type = std::uint32_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR5_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR5_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk1_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB1CLKEN_TMR5EN>(RCM->APB1CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::tmr5;
};

struct tim6 {
  static inline registers& reg = *TMR6;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 0;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB1CLKEN_TMR6EN>(RCM->APB1CLKEN);
  }
};

struct tim7 {
  static inline registers& reg = *TMR7;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 0;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB1CLKEN_TMR7EN>(RCM->APB1CLKEN);
  }
};

struct tim8 {
  static inline registers& reg = *TMR8;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 4;

  static constexpr nvic::irq_number update_irqn = TMR8_UP_TMR13_IRQn;
  static constexpr nvic::irq_number break_irqn = TMR8_BRK_TMR12_IRQn;
  static constexpr nvic::irq_number capture_compare_irqn = TMR8_CC_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk2_timer_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB2CLKEN_TMR8EN>(RCM->APB2CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::tmr8;
};

struct tim9 {
  static inline registers& reg = *TMR9;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 2;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB2CLKEN_TMR9EN>(RCM->APB2CLKEN);
  }
};

struct tim10 {
  static inline registers& reg = *TMR10;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB2CLKEN_TMR10EN>(RCM->APB2CLKEN);
  }
};

struct tim11 {
  static inline registers& reg = *TMR11;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB2CLKEN_TMR11EN>(RCM->APB2CLKEN);
  }
};

struct tim12 {
  static inline registers& reg = *TMR12;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 2;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB1CLKEN_TMR12EN>(RCM->APB1CLKEN);
  }
};

struct tim13 {
  static inline registers& reg = *TMR13;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB1CLKEN_TMR13EN>(RCM->APB1CLKEN);
  }
};

struct tim14 {
  static inline registers& reg = *TMR14;

  using counter_type = std::uint16_t;
  static constexpr unsigned io_channel_count = 1;

  static void enable_clock()
  {
    emb::mmio::set<RCM_APB1CLKEN_TMR14EN>(RCM->APB1CLKEN);
  }
};

template<typename T>
struct is_timer_instance : std::bool_constant<emb::same_as_any<T,
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
concept some_timer_instance = is_timer_instance<T>::value;

template<typename T>
struct is_advanced_timer : std::bool_constant<emb::same_as_any<T, tim1, tim8>> {
};

template<typename T>
concept some_advanced_timer = is_advanced_timer<T>::value;

template<typename T>
struct is_general_purpose_timer : std::bool_constant<emb::same_as_any<T,
                                                                      tim2,
                                                                      tim3,
                                                                      tim4,
                                                                      tim5,
                                                                      tim9,
                                                                      tim10,
                                                                      tim11,
                                                                      tim12,
                                                                      tim13,
                                                                      tim14>> {
};

template<typename T>
concept some_general_purpose_timer = is_general_purpose_timer<T>::value;

template<typename T>
struct is_32bit_timer
    : std::bool_constant<
          std::same_as<typename T::counter_type, std::uint32_t>> {};

template<typename T>
concept some_32bit_timer = is_32bit_timer<T>::value;

template<typename T>
struct is_basic_timer : std::bool_constant<emb::same_as_any<T, tim6, tim7>> {};

template<typename T>
concept some_basic_timer = is_basic_timer<T>::value;

template<typename T>
struct is_master_timer_instance
    : std::bool_constant<
          emb::same_as_any<T, tim1, tim2, tim3, tim4, tim5, tim6, tim7, tim8>> {
};

template<typename T>
concept some_master_timer_instance = is_master_timer_instance<T>::value;

enum class clock_division : std::uint32_t {
  div1 = 0b00u,
  div2 = 0b01u,
  div4 = 0b10u
};

enum class count_direction : std::uint32_t { up, down };

enum class counter_mode : std::uint32_t { up, down, updown };

enum class trigger_output : std::uint32_t {
  reset,
  enable,
  update,
  compare_pulse,
  oc1ref,
  oc2ref,
  oc3ref,
  oc4ref
};

template<some_timer_instance Tim>
void enable_counter()
{
  emb::mmio::set<TMR_CTRL1_CNTEN>(Tim::reg.CTRL1);
}

template<some_timer_instance Tim>
void disable_counter()
{
  emb::mmio::clear<TMR_CTRL1_CNTEN>(Tim::reg.CTRL1);
}

template<some_timer_instance Tim>
bool update_flag()
{
  return emb::mmio::test<TMR_STS_UIFLG>(Tim::reg.STS);
}

template<some_timer_instance Tim>
void acknowledge_update()
{
  emb::mmio::clear_w0<TMR_STS_UIFLG>(Tim::reg.STS);
}

template<some_timer_instance Tim>
bool break_flag()
{
  return emb::mmio::test<TMR_STS_BRKIFLG>(Tim::reg.STS);
}

template<some_timer_instance Tim>
void acknowledge_break()
{
  emb::mmio::clear_w0<TMR_STS_BRKIFLG>(Tim::reg.STS);
}

namespace detail {

template<some_timer_instance Tim>
constexpr std::uint16_t calculate_prescaler(emb::units::hz_f32 clk_freq,
                                            emb::units::hz_f32 tim_freq,
                                            counter_mode mode)
{
  std::uint32_t clk_freq_u32 = static_cast<std::uint32_t>(clk_freq.value);
  std::uint32_t tim_freq_u32 = static_cast<std::uint32_t>(tim_freq.value);

  // constexpr replacement for std::div (must be constrexpr since c++23, but...)
  std::uint32_t total_ticks =
      clk_freq_u32 / tim_freq_u32 + (clk_freq_u32 % tim_freq_u32 != 0) - 1;
  if (mode == counter_mode::updown) {
    total_ticks = (total_ticks + 1) / 2;
  }

  std::uint32_t ret =
      total_ticks / std::numeric_limits<typename Tim::counter_type>::max();
  emb::ensure(ret <= UINT16_MAX);

  return static_cast<std::uint16_t>(ret);
}

} // namespace detail

template<some_timer_instance Tim>
std::uint16_t calculate_prescaler(emb::units::hz_f32 tim_freq,
                                  counter_mode mode)
{
  return detail::calculate_prescaler<Tim>(
      Tim::template clock_frequency<emb::units::hz_f32>(),
      tim_freq,
      mode);
}

} // namespace apm32::f4::tim
