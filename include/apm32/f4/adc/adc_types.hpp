#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/dma.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_adc.h>
#include <apm32f4xx_rcm.h>

#include <concepts>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

using common_registers = ADC_Common_T;
using registers = ADC_T;

inline constexpr size_t count = 3;

inline constexpr emb::units::hz_f32 max_clock_frequency{30e6f};

struct adc1 {
  static inline common_registers& common_regs = *ADC;
  static inline registers& regs = *ADC1;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC1);
  };

  using dma_streams =
      emb::type_list<dma::v2::dma2_stream0, dma::v2::dma2_stream4>;
  using dma_channel = dma::v2::channel0;
};

struct adc2 {
  static inline common_registers& common_regs = *ADC;
  static inline registers& regs = *ADC2;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC2);
  };

  using dma_streams =
      emb::type_list<dma::v2::dma2_stream2, dma::v2::dma2_stream3>;
  using dma_channel = dma::v2::channel1;
};

struct adc3 {
  static inline common_registers& common_regs = *ADC;
  static inline registers& regs = *ADC3;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC3);
  };

  using dma_streams =
      emb::type_list<dma::v2::dma2_stream0, dma::v2::dma2_stream1>;
  using dma_channel = dma::v2::channel2;
};

template<typename T>
struct is_adc_module_instance
    : std::bool_constant<emb::same_as_any<T, adc1, adc2, adc3>> {};

template<typename T>
concept adc_module_instance = is_adc_module_instance<T>::value;

enum class external_trigger_edge : uint32_t { none, rising, falling, both };

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
