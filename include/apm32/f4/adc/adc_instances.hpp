#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/adc/adc_registers.hpp>

#include <apm32/f4/dma.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_adc.h>
#include <apm32f4xx_rcm.h>

#include <chrono>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

using common_registers = ADC_Common_T;
using registers = ADC_T;

inline constexpr size_t count = 3;

inline constexpr emb::units::hz_f32 max_clock_frequency{30e6f};
inline constexpr std::chrono::microseconds powerup_time{3};
inline constexpr float vref = 3.3f;

template<typename T>
inline constexpr T nmax = T{4095};

struct adc1 {
  static constexpr uintptr_t base_addr = ADC1_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline common_registers& common_regs = *ADC;
  static inline registers& regs = *ADC1;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC1);
  };

  using dma_streams =
      emb::typelist<dma::v2::dma2_stream0, dma::v2::dma2_stream4>;
  using dma_channel = dma::v2::channel0;
};

struct adc2 {
  static constexpr uintptr_t base_addr = ADC2_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline common_registers& common_regs = *ADC;
  static inline registers& regs = *ADC2;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC2);
  };

  using dma_streams =
      emb::typelist<dma::v2::dma2_stream2, dma::v2::dma2_stream3>;
  using dma_channel = dma::v2::channel1;
};

struct adc3 {
  static constexpr uintptr_t base_addr = ADC3_BASE;
  using reg_addr = detail::register_addresses<base_addr>;

  static inline common_registers& common_regs = *ADC;
  static inline registers& regs = *ADC3;
  static constexpr nvic::irq_number irqn = ADC_IRQn;

  static constexpr auto enable_clock = []() {
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC3);
  };

  using dma_streams =
      emb::typelist<dma::v2::dma2_stream0, dma::v2::dma2_stream1>;
  using dma_channel = dma::v2::channel2;
};

template<typename T>
struct is_adc_instance
    : std::bool_constant<emb::same_as_any<T, adc1, adc2, adc3>> {};

template<typename T>
concept adc_instance = is_adc_instance<T>::value;

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
