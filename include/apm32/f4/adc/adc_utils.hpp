#pragma once

#include <apm32/f4/adc/adc_instances.hpp>

#include <emb/units.hpp>

#include <algorithm>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

template<adc_instance Instance>
void start_injected() {
  registers& regs = Instance::regs;
  regs.CTRL2_B.INJSWSC = 1;
}

template<adc_instance Instance>
void start_regular() {
  registers& regs = Instance::regs;
  regs.CTRL2_B.REGSWSC = 1;
}

template<adc_instance Instance>
bool jeoc_flag() {
  registers const& regs = Instance::regs;
  return regs.STS_B.INJEOCFLG == 1;
}

template<adc_instance Instance>
void acknowledge_jeoc() {
  registers& regs = Instance::regs;
  regs.STS_B.INJEOCFLG = 0;
}

template<adc_instance Instance>
bool eoc_flag() {
  registers const& regs = Instance::regs;
  return regs.STS_B.EOCFLG == 1;
}

template<adc_instance Instance>
void acknowledge_eoc() {
  registers& regs = Instance::regs;
  regs.STS_B.EOCFLG = 0;
}

inline constexpr std::array<uint32_t, 4> clock_prescalers = {2, 4, 6, 8};

namespace detail {

constexpr ADC_PRESCALER_T to_sdk(uint32_t prescaler) {
  core::ensure(
      std::find(clock_prescalers.begin(), clock_prescalers.end(), prescaler) !=
      clock_prescalers.end()
  );

  switch (prescaler) {
  case 2:
    return ADC_PRESCALER_DIV2;
  case 4:
    return ADC_PRESCALER_DIV4;
  case 6:
    return ADC_PRESCALER_DIV6;
  case 8:
    return ADC_PRESCALER_DIV8;
  }
  std::unreachable();
}

constexpr uint32_t
calculate_prescaler(emb::units::hz_f32 clk_freq, emb::units::hz_f32 adc_freq) {
  uint32_t clk_freq_u32 = static_cast<uint32_t>(clk_freq.value());
  uint32_t adc_freq_u32 = static_cast<uint32_t>(adc_freq.value());

  // constexpr replacement for std::div (must be constrexpr since c++23, but...)
  uint32_t ratio = clk_freq_u32 / adc_freq_u32 +
                   (clk_freq_u32 % adc_freq_u32 != 0);
  auto it = std::upper_bound(
      clock_prescalers.begin(),
      clock_prescalers.end(),
      ratio
  );
  core::ensure(it != clock_prescalers.end());
  return *it;
}

} // namespace detail

inline uint32_t calculate_prescaler() {
  return detail::calculate_prescaler(
      core::apb2_timer_frequency<emb::units::hz_f32>(),
      max_clock_frequency
  );
}

inline float convert_to_mcu_temperature(uint32_t adc_data) {
  float const volt = static_cast<float>(adc_data) * vref / nmax<float>;
  return (volt - 0.7782f) / 0.0024f + 28.0f;
}

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
