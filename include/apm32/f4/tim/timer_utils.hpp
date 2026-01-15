#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>

#include <emb/units.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

namespace detail {

constexpr TMR_CLOCK_DIV_T to_sdk(clock_division clkdiv) {
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

template<timer_instance Tim>
constexpr uint16_t calculate_prescaler(
    emb::units::hz_f32 clk_freq,
    emb::units::hz_f32 tim_freq,
    counter_mode mode
) {
  uint32_t clk_freq_u32 = static_cast<uint32_t>(clk_freq.value());
  uint32_t tim_freq_u32 = static_cast<uint32_t>(tim_freq.value());

  // constexpr replacement for std::div (must be constrexpr since c++23, but...)
  uint32_t total_ticks = clk_freq_u32 / tim_freq_u32 +
                         (clk_freq_u32 % tim_freq_u32 != 0) -
                         1;
  if (mode == counter_mode::updown) {
    total_ticks = (total_ticks + 1) / 2;
  }

  uint32_t ret = total_ticks /
                 std::numeric_limits<typename Tim::counter_type>::max();
  core::ensure(ret <= UINT16_MAX);

  return static_cast<uint16_t>(ret);
}

} // namespace detail

template<timer_instance Tim>
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
