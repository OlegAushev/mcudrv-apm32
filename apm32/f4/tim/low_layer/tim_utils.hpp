#pragma once

#include <apm32/f4/tim/low_layer/tim_channels.hpp>
#include <apm32/f4/tim/low_layer/tim_instances.hpp>
#include <apm32/f4/tim/low_layer/tim_types.hpp>

#include <emb/assert.hpp>
#include <emb/mmio.hpp>
#include <emb/units.hpp>

#include <cstdint>

namespace apm32::f4::tim {

template<some_timer_instance Tim>
void enable_counter() {
  emb::mmio::set<TMR_CTRL1_CNTEN>(Tim::REG.CTRL1);
}

template<some_timer_instance Tim>
void disable_counter() {
  emb::mmio::clear<TMR_CTRL1_CNTEN>(Tim::REG.CTRL1);
}

template<some_timer_instance Tim>
bool update_flag() {
  return emb::mmio::test<TMR_STS_UIFLG>(Tim::REG.STS);
}

template<some_timer_instance Tim>
void acknowledge_update() {
  emb::mmio::clear_w0<TMR_STS_UIFLG>(Tim::REG.STS);
}

template<some_timer_instance Tim>
bool break_flag() {
  return emb::mmio::test<TMR_STS_BRKIFLG>(Tim::REG.STS);
}

template<some_timer_instance Tim>
void acknowledge_break() {
  emb::mmio::clear_w0<TMR_STS_BRKIFLG>(Tim::REG.STS);
}

template<some_timer_instance Tim, some_timer_channel_instance Ch>
bool capture_compare_flag() {
  if constexpr (std::same_as<Ch, channel1>) {
    return emb::mmio::test<TMR_STS_CC1IFLG>(Tim::REG.STS);
  } else if constexpr (std::same_as<Ch, channel2>) {
    return emb::mmio::test<TMR_STS_CC2IFLG>(Tim::REG.STS);
  } else if constexpr (std::same_as<Ch, channel3>) {
    return emb::mmio::test<TMR_STS_CC3IFLG>(Tim::REG.STS);
  } else if constexpr (std::same_as<Ch, channel4>) {
    return emb::mmio::test<TMR_STS_CC4IFLG>(Tim::REG.STS);
  } else {
    std::unreachable();
  }
}

template<some_timer_instance Tim, some_timer_channel_instance Ch>
void acknowledge_capture_compare() {
  if constexpr (std::same_as<Ch, channel1>) {
    emb::mmio::clear_w0<TMR_STS_CC1IFLG>(Tim::REG.STS);
  } else if constexpr (std::same_as<Ch, channel2>) {
    emb::mmio::clear_w0<TMR_STS_CC2IFLG>(Tim::REG.STS);
  } else if constexpr (std::same_as<Ch, channel3>) {
    emb::mmio::clear_w0<TMR_STS_CC3IFLG>(Tim::REG.STS);
  } else if constexpr (std::same_as<Ch, channel4>) {
    emb::mmio::clear_w0<TMR_STS_CC4IFLG>(Tim::REG.STS);
  } else {
    std::unreachable();
  }
}

namespace detail {

template<some_timer_instance Tim>
constexpr std::uint16_t calculate_prescaler(
    emb::units::hz_f32 clk_freq,
    emb::units::hz_f32 tim_freq,
    counter_mode mode
) {
  std::uint32_t clk_freq_u32 = static_cast<std::uint32_t>(clk_freq.value());
  std::uint32_t tim_freq_u32 = static_cast<std::uint32_t>(tim_freq.value());

  // constexpr replacement for std::div (must be constrexpr since c++23, but...)
  std::uint32_t total_ticks = clk_freq_u32 / tim_freq_u32
                            + (clk_freq_u32 % tim_freq_u32 != 0)
                            - 1;
  if (mode == counter_mode::updown) {
    total_ticks = (total_ticks + 1) / 2;
  }

  std::uint32_t ret = total_ticks
                    / std::numeric_limits<typename Tim::counter_type>::max();
  emb::ensure(ret <= UINT16_MAX);

  return static_cast<std::uint16_t>(ret);
}

} // namespace detail

template<some_timer_instance Tim>
std::uint16_t
calculate_prescaler(emb::units::hz_f32 tim_freq, counter_mode mode) {
  return detail::calculate_prescaler<Tim>(
      Tim::template clock_frequency<emb::units::hz_f32>(),
      tim_freq,
      mode
  );
}

} // namespace apm32::f4::tim
