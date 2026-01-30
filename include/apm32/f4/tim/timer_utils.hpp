#pragma once

#include <apm32/f4/tim/timer_channels.hpp>
#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>

#include <emb/units.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

template<timer_instance Tim>
void enable_counter() {
  registers& regs = Tim::regs;
  regs.CTRL1_B.CNTEN = 1;
}

template<timer_instance Tim>
void disable_counter() {
  registers& regs = Tim::regs;
  regs.CTRL1_B.CNTEN = 0;
}

template<timer_instance Tim>
bool update_flag() {
  registers const& regs = Tim::regs;
  return regs.STS_B.UIFLG == 1;
}

template<timer_instance Tim>
void acknowledge_update() {
  registers& regs = Tim::regs;
  regs.STS_B.UIFLG = 0;
}

template<timer_instance Tim>
bool break_flag() {
  registers const& regs = Tim::regs;
  return regs.STS_B.BRKIFLG == 1;
}

template<timer_instance Tim>
void acknowledge_break() {
  registers& regs = Tim::regs;
  regs.STS_B.BRKIFLG = 0;
}

template<timer_instance Tim, timer_channel_instance Ch>
bool capture_compare_flag() {
  registers& regs = Tim::regs;
  if constexpr (std::same_as<Ch, channel1>) {
    return regs.STS_B.CC1IFLG;
  } else if constexpr (std::same_as<Ch, channel2>) {
    return regs.STS_B.CC2IFLG;
  } else if constexpr (std::same_as<Ch, channel3>) {
    return regs.STS_B.CC3IFLG;
  } else if constexpr (std::same_as<Ch, channel4>) {
    return regs.STS_B.CC4IFLG;
  } else {
    std::unreachable();
  }
}

template<timer_instance Tim, timer_channel_instance Ch>
void acknowledge_capture_compare() {
  registers& regs = Tim::regs;
  if constexpr (std::same_as<Ch, channel1>) {
    regs.STS_B.CC1IFLG = 0;
  } else if constexpr (std::same_as<Ch, channel2>) {
    regs.STS_B.CC2IFLG = 0;
  } else if constexpr (std::same_as<Ch, channel3>) {
    regs.STS_B.CC3IFLG = 0;
  } else if constexpr (std::same_as<Ch, channel4>) {
    regs.STS_B.CC4IFLG = 0;
  } else {
    std::unreachable();
  }
}

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
