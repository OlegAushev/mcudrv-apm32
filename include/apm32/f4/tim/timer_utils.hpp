#pragma once

#include <apm32/f4/tim/timer_channels.hpp>
#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>

#include <emb/mmio.hpp>
#include <emb/units.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

template<timer_instance Tim>
void enable_counter() {
  registers& regs = Tim::regs;
  emb::mmio::set(regs.CTRL1, TMR_CTRL1_CNTEN);
}

template<timer_instance Tim>
void disable_counter() {
  registers& regs = Tim::regs;
  emb::mmio::clear(regs.CTRL1, TMR_CTRL1_CNTEN);
}

template<timer_instance Tim>
bool update_flag() {
  registers const& regs = Tim::regs;
  return emb::mmio::test_any(regs.STS, TMR_STS_UIFLG);
}

template<timer_instance Tim>
void acknowledge_update() {
  registers& regs = Tim::regs;
  emb::mmio::clear_w0(regs.STS, TMR_STS_UIFLG);
}

template<timer_instance Tim>
bool break_flag() {
  registers const& regs = Tim::regs;
  return emb::mmio::test_any(regs.STS, TMR_STS_BRKIFLG);
}

template<timer_instance Tim>
void acknowledge_break() {
  registers& regs = Tim::regs;
  emb::mmio::clear_w0(regs.STS, TMR_STS_BRKIFLG);
}

template<timer_instance Tim, timer_channel_instance Ch>
bool capture_compare_flag() {
  registers& regs = Tim::regs;
  if constexpr (std::same_as<Ch, channel1>) {
    return emb::mmio::test_any(regs.STS, TMR_STS_CC1IFLG);
  } else if constexpr (std::same_as<Ch, channel2>) {
    return emb::mmio::test_any(regs.STS, TMR_STS_CC2IFLG);
  } else if constexpr (std::same_as<Ch, channel3>) {
    return emb::mmio::test_any(regs.STS, TMR_STS_CC3IFLG);
  } else if constexpr (std::same_as<Ch, channel4>) {
    return emb::mmio::test_any(regs.STS, TMR_STS_CC4IFLG);
  } else {
    std::unreachable();
  }
}

template<timer_instance Tim, timer_channel_instance Ch>
void acknowledge_capture_compare() {
  registers& regs = Tim::regs;
  if constexpr (std::same_as<Ch, channel1>) {
    emb::mmio::clear_w0(regs.STS, TMR_STS_CC1IFLG);
  } else if constexpr (std::same_as<Ch, channel2>) {
    emb::mmio::clear_w0(regs.STS, TMR_STS_CC2IFLG);
  } else if constexpr (std::same_as<Ch, channel3>) {
    emb::mmio::clear_w0(regs.STS, TMR_STS_CC3IFLG);
  } else if constexpr (std::same_as<Ch, channel4>) {
    emb::mmio::clear_w0(regs.STS, TMR_STS_CC4IFLG);
  } else {
    std::unreachable();
  }
}

namespace detail {

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
