#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/dma.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_adc.h>

#include <memory>
#include <optional>
#include <utility>

namespace apm32 {
namespace f4 {
namespace adc {

using common_registers = ADC_Common_T;
using peripheral_registers = ADC_T;

inline constexpr float vref = 3.3f;

template<typename T>
inline constexpr T nmax = T{4095};

inline constexpr size_t peripheral_count = 3;

enum class peripheral_id : uint32_t { adc1, adc2, adc3 };

struct pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
};

struct injected_config {
  uint8_t conv_num;
  bool auto_conv;
  bool discontinuous_mode;
  ADC_EXT_TRIG_INJEC_EDGE_T ext_trigger_edge;
  ADC_EXT_TRIG_INJEC_CONV_T ext_trigger;
};

struct config {
  ADC_CommonConfig_T hal_common_config;
  ADC_RESOLUTION_T resolution;
  bool scan_mode;
  bool continuous_mode;
  ADC_EXT_TRIG_EDGE_T ext_trigger_edge;
  ADC_EXT_TRIG_CONV_T ext_trigger;
  ADC_DATA_ALIGN_T data_align;
  uint8_t conv_num;
  bool eoc_on_each_conv;
  bool dma_continuous_requests;
  bool discontinuous_mode;
  uint8_t discontinuous_conv_num;
  std::optional<injected_config> injected;
};

struct regular_channel_config {
  ADC_CHANNEL_T channel;
  ADC_SAMPLETIME_T sampletime;
  std::initializer_list<uint8_t> ranks;
};

enum class injected_channel_rank : uint32_t { rank1, rank2, rank3, rank4 };

struct injected_channel_config {
  ADC_CHANNEL_T channel;
  ADC_SAMPLETIME_T sampletime;
  std::initializer_list<injected_channel_rank> ranks;
  uint16_t offset;
};

namespace detail {

inline std::array<peripheral_registers*, peripheral_count> const peripherals =
    {ADC1, ADC2, ADC3};

inline std::array<void (*)(void), peripheral_count> const enable_clock = {
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC1); },
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC2); },
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC3); }
};

inline std::array<ADC_INJEC_CHANNEL_T, 4> injected_selection = {
    ADC_INJEC_CHANNEL_1,
    ADC_INJEC_CHANNEL_2,
    ADC_INJEC_CHANNEL_3,
    ADC_INJEC_CHANNEL_4
};

} // namespace detail

class peripheral
    : public apm32::peripheral<peripheral, peripheral_id, peripheral_count> {
public:
  using peripheral_type =
      apm32::peripheral<peripheral, peripheral_id, peripheral_count>;
private:
  peripheral_id const id_;
  peripheral_registers* const reg_;
  static inline common_registers* common_reg_ = nullptr;
public:
  peripheral(
      peripheral_id id,
      config const& conf,
      dma::stream* dma_stream = nullptr
  );

  peripheral_id id() const {
    return id_;
  }

  peripheral_registers* reg() {
    return reg_;
  }

  common_registers* common_reg() {
    return common_reg_;
  }

  [[nodiscard]] std::unique_ptr<gpio::analog_pin> configure_injected(
      pin_config const& pinconf,
      injected_channel_config const& chconf
  );

  [[nodiscard]] std::unique_ptr<gpio::analog_pin> configure_regular(
      pin_config const& pinconf,
      regular_channel_config const& chconf
  );

  void configure_internal_injected(injected_channel_config const& chconf);

  void configure_internal_regular(regular_channel_config const& chconf);

  void start_injected() {
    if (reg_->STS_B.INJCSFLG == 1) {
      return; // there is ongoing injected channel conversion
    }
    reg_->CTRL2_B.INJSWSC = 1;
  }

  bool injected_busy() const {
    return (reg_->STS_B.INJCSFLG == 1);
  }

  bool injected_ready() const {
    return (reg_->STS_B.INJEOCFLG == 1);
  }

  uint32_t read_injected(injected_channel_rank rank) {
    switch (rank) {
    case injected_channel_rank::rank1:
      return reg_->INJDATA1;
    case injected_channel_rank::rank2:
      return reg_->INJDATA2;
    case injected_channel_rank::rank3:
      return reg_->INJDATA3;
    case injected_channel_rank::rank4:
      return reg_->INJDATA4;
    }
    std::unreachable();
  }

  void ack_injected() {
    reg_->STS_B.INJCSFLG = 0;
    reg_->STS_B.INJEOCFLG = 0;
  }

  void start_regular() {
    if (reg_->STS_B.REGCSFLG == 1) {
      return; // there is ongoing regular channel conversion
    }
    reg_->CTRL2_B.REGSWSC = 1;
  }

  bool regular_busy() const {
    return (reg_->STS_B.REGCSFLG == 1);
  }

  bool regular_ready() const {
    return (reg_->STS_B.EOCFLG == 1);
  }

  uint32_t read_regular() {
    return reg_->REGDATA;
  }

  void ack_regular() {
    reg_->STS_B.REGCSFLG = 0;
    reg_->STS_B.EOCFLG = 0;
  }

public:
  void configure_interrupts(uint32_t interrupt_bitset);

  static void set_interrupts_priority(nvic::IrqPriority priority) {
    nvic::set_irq_priority(ADC_IRQn, priority);
  }

  static void enable_interrupts() {
    nvic::enable_irq(ADC_IRQn);
  }

  static void disable_interrupts() {
    nvic::disable_irq(ADC_IRQn);
  }

  bool has_interrupt(ADC_INT_T interrupt) const {
    auto sts = reg_->STS_B;
    auto cr1 = reg_->CTRL1_B;

    switch (interrupt) {
    case ADC_INT_EOC:
      return sts.EOCFLG && cr1.EOCIEN;
      break;
    case ADC_INT_AWD:
      return sts.AWDFLG && cr1.AWDIEN;
      break;
    case ADC_INT_INJEOC:
      return sts.INJEOCFLG && cr1.INJEOCIEN;
      break;
    case ADC_INT_OVR:
      return sts.OVREFLG && cr1.OVRIEN;
      break;
    }
    return false;
  }
private:
  static inline std::array<bool, peripheral_count> is_clock_enabled_{};
  static void enable_clock(peripheral_id id);
};

} // namespace adc
} // namespace f4
} // namespace apm32
