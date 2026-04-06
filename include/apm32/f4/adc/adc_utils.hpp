#pragma once

#include <apm32/f4/adc/adc_instances.hpp>

#include <emb/mmio.hpp>
#include <emb/units.hpp>

#include <algorithm>

namespace apm32 {
namespace f4 {
namespace adc {

template<adc_instance Instance>
void start_injected() {
  registers& regs = Instance::regs;
  emb::mmio::set(regs.CTRL2, ADC_CTRL2_INJSWSC);
}

template<adc_instance Instance>
void start_regular() {
  registers& regs = Instance::regs;
  emb::mmio::set(regs.CTRL2, ADC_CTRL2_REGSWSC);
}

template<adc_instance Instance>
bool jeoc_flag() {
  registers const& regs = Instance::regs;
  return emb::mmio::test_any(regs.STS, ADC_STS_INJEOCFLG);
}

template<adc_instance Instance>
void acknowledge_jeoc() {
  registers& regs = Instance::regs;
  emb::mmio::clear_w0(regs.STS, ADC_STS_INJEOCFLG);
}

template<adc_instance Instance>
bool eoc_flag() {
  registers const& regs = Instance::regs;
  return emb::mmio::test_any(regs.STS, ADC_STS_EOCFLG);
}

template<adc_instance Instance>
void acknowledge_eoc() {
  registers& regs = Instance::regs;
  emb::mmio::clear_w0(regs.STS, ADC_STS_EOCFLG);
}

inline constexpr std::array<uint32_t, 4> clock_prescalers = {2, 4, 6, 8};

namespace detail {

// prescaler field value: 0=div2, 1=div4, 2=div6, 3=div8
constexpr uint32_t prescaler_to_field(uint32_t prescaler) {
  switch (prescaler) {
  case 2:
    return 0;
  case 4:
    return 1;
  case 6:
    return 2;
  case 8:
    return 3;
  }
  std::unreachable();
}

constexpr uint32_t
calculate_prescaler(emb::units::hz_f32 clk_freq, emb::units::hz_f32 adc_freq) {
  uint32_t clk_freq_u32 = static_cast<uint32_t>(clk_freq.value());
  uint32_t adc_freq_u32 = static_cast<uint32_t>(adc_freq.value());

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

} // namespace adc
} // namespace f4
} // namespace apm32
