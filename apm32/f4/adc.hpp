#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/dma.hpp>
#include <apm32/f4/nvic.hpp>
#include <apm32/f4/rcc.hpp>

#include <emb/assert.hpp>
#include <emb/mmio.hpp>
#include <emb/units.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace apm32::f4::adc {

using common_registers = ADC_Common_TypeDef;
using registers = ADC_TypeDef;

inline constexpr std::size_t count = 3;

inline constexpr emb::units::hz_f32 max_clock_frequency{30e6f};
inline constexpr std::chrono::microseconds powerup_time{3};

inline constexpr float vref = 3.3f;
inline constexpr unsigned resolution = 12;

template<typename T>
inline constexpr T full_scale = T{1u << resolution};

template<typename T>
inline constexpr T max_code = full_scale<T> - 1;

inline constexpr float lsb = vref / full_scale<float>;
inline constexpr float codes_per_volt = 1 / lsb;

struct adc1 {
  static inline common_registers& common_reg = *ADC123_COMMON;
  static inline registers& reg = *ADC1;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB2CLKEN_ADC1EN>(RCM->APB2CLKEN);
  };

  using dma_streams = emb::typelist<dma::dma2_stream0, dma::dma2_stream4>;
  using dma_channel = dma::channel0;
};

struct adc2 {
  static inline common_registers& common_reg = *ADC123_COMMON;
  static inline registers& reg = *ADC2;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB2CLKEN_ADC2EN>(RCM->APB2CLKEN);
  };

  using dma_streams = emb::typelist<dma::dma2_stream2, dma::dma2_stream3>;
  using dma_channel = dma::channel1;
};

struct adc3 {
  static inline common_registers& common_reg = *ADC123_COMMON;
  static inline registers& reg = *ADC3;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB2CLKEN_ADC3EN>(RCM->APB2CLKEN);
  };

  using dma_streams = emb::typelist<dma::dma2_stream0, dma::dma2_stream1>;
  using dma_channel = dma::channel2;
};

template<typename T>
struct is_adc_instance
    : std::bool_constant<emb::same_as_any<T, adc1, adc2, adc3>> {};

template<typename T>
concept some_adc_instance = is_adc_instance<T>::value;

template<some_adc_instance Instance, typename T>
consteval bool is_compatible_dma_stream()
{
  return emb::typelist_contains_v<typename Instance::dma_streams, T>;
}

template<some_adc_instance Instance, typename T>
consteval bool is_compatible_dma_channel()
{
  return std::same_as<T, typename Instance::dma_channel>;
}

enum class trigger_edge : std::uint32_t {
  rising = 0b01,
  falling = 0b10,
  both = 0b11
};

enum class inj_trigger_event : std::uint32_t {
  tim1_cc4 = 0b0000u,
  tim1_trgo = 0b0001u,
  tim2_cc1 = 0b0010u,
  tim2_trgo = 0b0011u,
  tim3_cc2 = 0b0100u,
  tim3_cc4 = 0b0101u,
  tim4_cc1 = 0b0110u,
  tim4_cc2 = 0b0111u,
  tim4_cc3 = 0b1000u,
  tim4_trgo = 0b1001u,
  tim5_cc4 = 0b1010u,
  tim5_trgo = 0b1011u,
  tim8_cc2 = 0b1100u,
  tim8_cc3 = 0b1101u,
  tim8_cc4 = 0b1110u,
  exti_line15 = 0b1111u
};

enum class reg_trigger_event : std::uint32_t {
  tim1_cc1 = 0b0000u,
  tim1_cc2 = 0b0001u,
  tim1_cc3 = 0b0010u,
  tim2_cc2 = 0b0011u,
  tim2_cc3 = 0b0100u,
  tim2_cc4 = 0b0101u,
  tim2_trgo = 0b0110u,
  tim3_cc1 = 0b0111u,
  tim3_trgo = 0b1000u,
  tim4_cc4 = 0b1001u,
  tim5_cc1 = 0b1010u,
  tim5_cc2 = 0b1011u,
  tim5_cc3 = 0b1100u,
  tim8_cc1 = 0b1101u,
  tim8_trgo = 0b1110u,
  exti_line11 = 0b1111u
};

struct inj_trigger {
  trigger_edge edge;
  inj_trigger_event event;
};

struct reg_trigger {
  trigger_edge edge;
  reg_trigger_event event;
};

template<some_adc_instance Instance>
void start_injected()
{
  emb::mmio::set<ADC_CTRL2_INJSWSC>(Instance::reg.CTRL2);
}

template<some_adc_instance Instance>
void start_regular()
{
  emb::mmio::set<ADC_CTRL2_REGSWSC>(Instance::reg.CTRL2);
}

template<some_adc_instance Instance>
bool jeoc_flag()
{
  return emb::mmio::test<ADC_STS_INJEOCFLG>(Instance::reg.STS);
}

template<some_adc_instance Instance>
void acknowledge_jeoc()
{
  emb::mmio::clear_w0<ADC_STS_INJEOCFLG>(Instance::reg.STS);
}

template<some_adc_instance Instance>
bool eoc_flag()
{
  return emb::mmio::test<ADC_STS_EOCFLG>(Instance::reg.STS);
}

template<some_adc_instance Instance>
void acknowledge_eoc()
{
  emb::mmio::clear_w0<ADC_STS_EOCFLG>(Instance::reg.STS);
}

inline constexpr std::array<std::uint32_t, 4> clock_prescalers = {2, 4, 6, 8};

namespace detail {

// prescaler field value: 0=div2, 1=div4, 2=div6, 3=div8
constexpr std::uint32_t prescaler_to_field(std::uint32_t prescaler)
{
  switch (prescaler) {
  case 2: return 0;
  case 4: return 1;
  case 6: return 2;
  case 8: return 3;
  }
  std::unreachable();
}

constexpr std::uint32_t calculate_prescaler(emb::units::hz_f32 clk_freq,
                                            emb::units::hz_f32 adc_freq)
{
  std::uint32_t clk_freq_u32 = static_cast<std::uint32_t>(clk_freq.value);
  std::uint32_t adc_freq_u32 = static_cast<std::uint32_t>(adc_freq.value);

  std::uint32_t ratio =
      clk_freq_u32 / adc_freq_u32 + (clk_freq_u32 % adc_freq_u32 != 0);
  auto it =
      std::lower_bound(clock_prescalers.begin(), clock_prescalers.end(), ratio);
  emb::ensure(it != clock_prescalers.end());
  return *it;
}

} // namespace detail

inline std::uint32_t calculate_prescaler()
{
  return detail::calculate_prescaler(
      rcc::pclk2_frequency<emb::units::hz_f32>(),
      max_clock_frequency);
}

inline nvic::irq_priority common_irq_priority{0};

namespace detail {
void init_common();
} // namespace detail

} // namespace apm32::f4::adc
