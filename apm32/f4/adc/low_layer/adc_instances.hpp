#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/dma/dma.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/mmio.hpp>

#include <chrono>

namespace apm32::f4::adc {

using common_registers = ADC_Common_TypeDef;
using registers = ADC_TypeDef;

inline constexpr size_t count = 3;

inline constexpr emb::units::hz_f32 max_clock_frequency{30e6f};
inline constexpr std::chrono::microseconds powerup_time{3};
inline constexpr float vref = 3.3f;

template<typename T>
inline constexpr T nmax = T{4095};

struct adc1 {
  static inline common_registers& common_reg = *ADC123_COMMON;
  static inline registers& reg = *ADC1;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC1EN);
  };

  using dma_streams = emb::typelist<dma::dma2_stream0, dma::dma2_stream4>;
  using dma_channel = dma::channel0;
};

struct adc2 {
  static inline common_registers& common_reg = *ADC123_COMMON;
  static inline registers& reg = *ADC2;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC2EN);
  };

  using dma_streams = emb::typelist<dma::dma2_stream2, dma::dma2_stream3>;
  using dma_channel = dma::channel1;
};

struct adc3 {
  static inline common_registers& common_reg = *ADC123_COMMON;
  static inline registers& reg = *ADC3;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC3EN);
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
consteval bool is_compatible_dma_stream() {
  return emb::typelist_contains_v<typename Instance::dma_streams, T>;
}

template<some_adc_instance Instance, typename T>
consteval bool is_compatible_dma_channel() {
  return std::same_as<T, typename Instance::dma_channel>;
}

} // namespace apm32::f4::adc
