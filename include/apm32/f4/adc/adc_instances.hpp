#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/adc/adc_registers.hpp>

#include <apm32/f4/dma.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/mmio.hpp>

#include <chrono>

namespace apm32 {
namespace f4 {
namespace adc {

using common_registers = ADC_Common_TypeDef;
using registers = ADC_TypeDef;

inline constexpr size_t count = 3;

inline constexpr emb::units::hz_f32 max_clock_frequency{30e6f};
inline constexpr std::chrono::microseconds powerup_time{3};
inline constexpr float vref = 3.3f;

template<typename T>
inline constexpr T nmax = T{4095};

struct adc1 {
  static constexpr uintptr_t base_addr = ADC1_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline common_registers& common_regs = *ADC123_COMMON;
  static inline registers& regs = *ADC1;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC1EN);
  };

  using dma_streams = emb::typelist<dma::dma2_stream0, dma::dma2_stream4>;
  using dma_channel = dma::channel0;
};

struct adc2 {
  static constexpr uintptr_t base_addr = ADC2_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline common_registers& common_regs = *ADC123_COMMON;
  static inline registers& regs = *ADC2;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_ADC2EN);
  };

  using dma_streams = emb::typelist<dma::dma2_stream2, dma::dma2_stream3>;
  using dma_channel = dma::channel1;
};

struct adc3 {
  static constexpr uintptr_t base_addr = ADC3_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline common_registers& common_regs = *ADC123_COMMON;
  static inline registers& regs = *ADC3;
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
concept adc_instance = is_adc_instance<T>::value;

} // namespace adc
} // namespace f4
} // namespace apm32
