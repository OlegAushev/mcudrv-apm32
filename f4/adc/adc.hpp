#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_adc.h>

#include <mcudrv-apm32/f4/dma/dma.hpp>
#include <mcudrv-apm32/f4/gpio/gpio.hpp>
#include <mcudrv-apm32/f4/system/system.hpp>

#include <emb/singleton.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace mcu {
inline namespace apm32 {
namespace adc {

using CommonRegs = ADC_Common_T;
using Regs = ADC_T;

constexpr float vref() {
  return 3.3f;
}

template<typename T>
constexpr T nmax() {
  return T{4095};
}

constexpr size_t periph_num{3};

enum class Peripheral : size_t {
  adc1,
  adc2,
  adc3
};

inline std::array<Regs*, periph_num> const regs{ADC1, ADC2, ADC3};

inline Peripheral get_peripheral(Regs const* reg) {
  return static_cast<Peripheral>(
      std::distance(regs.begin(), std::find(regs.begin(), regs.end(), reg)));
}

struct PinConfig {
  gpio::Port port;
  gpio::Pin pin;
};

struct InjectedConfig {
  uint8_t conv_num;
  bool auto_conv;
  bool discontinuous_mode;
  ADC_EXT_TRIG_INJEC_EDGE_T ext_trigger_edge;
  ADC_EXT_TRIG_INJEC_CONV_T ext_trigger;
};

struct Config {
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
  std::optional<InjectedConfig> injected;
};

struct RegularChannelConfig {
  ADC_CHANNEL_T channel;
  ADC_SAMPLETIME_T sampletime;
  std::initializer_list<uint8_t> ranks;
};

enum class InjectedChannelRank {
  rank1 = ADC_INJEC_CHANNEL_1,
  rank2 = ADC_INJEC_CHANNEL_2,
  rank3 = ADC_INJEC_CHANNEL_3,
  rank4 = ADC_INJEC_CHANNEL_4
};

struct InjectedChannelConfig {
  ADC_CHANNEL_T channel;
  ADC_SAMPLETIME_T sampletime;
  std::initializer_list<InjectedChannelRank> ranks;
  uint16_t offset;
};

class Module : public emb::singleton_array<Module, periph_num>,
               private emb::noncopyable {
private:
  Peripheral const peripheral_;
  Regs* const regs_;
  static inline CommonRegs* common_regs_{nullptr};
public:
  Module(Peripheral peripheral, Config const& conf, dma::Stream* dma = nullptr);

  Peripheral peripheral() const { return peripheral_; }

  Regs* regs() { return regs_; }

  CommonRegs* common_regs() { return common_regs_; }

  static Module* instance(Peripheral peripheral) {
    return emb::singleton_array<Module, periph_num>::instance(
        std::to_underlying(peripheral));
  }

  [[nodiscard]] std::unique_ptr<gpio::AnalogPin>
  init_injected(PinConfig const& pinconf, InjectedChannelConfig const& chconf);

  [[nodiscard]] std::unique_ptr<gpio::AnalogPin>
  init_regular(PinConfig const& pinconf, RegularChannelConfig const& chconf);

  void init_internal_injected(InjectedChannelConfig const& chconf);
  void init_internal_regular(RegularChannelConfig const& chconf);

  void start_injected() {
    if (regs_->STS_B.INJCSFLG == 1) {
      return; // there is ongoing injected channel conversion
    }
    regs_->CTRL2_B.INJSWSC = 1;
  }

  bool injected_busy() const { return (regs_->STS_B.INJCSFLG == 1); }

  bool injected_ready() const { return (regs_->STS_B.INJEOCFLG == 1); }

  uint32_t read_injected(InjectedChannelRank rank) {
    switch (rank) {
    case InjectedChannelRank::rank1:
      return regs_->INJDATA1;
    case InjectedChannelRank::rank2:
      return regs_->INJDATA2;
    case InjectedChannelRank::rank3:
      return regs_->INJDATA3;
    case InjectedChannelRank::rank4:
      return regs_->INJDATA4;
    }
    return 0xFFFFFFFF;
  }

  void ack_injected() {
    regs_->STS_B.INJCSFLG = 0;
    regs_->STS_B.INJEOCFLG = 0;
  }

  void start_regular() {
    if (regs_->STS_B.REGCSFLG == 1) {
      return; // there is ongoing regular channel conversion
    }
    regs_->CTRL2_B.REGSWSC = 1;
  }

  bool regular_busy() const { return (regs_->STS_B.REGCSFLG == 1); }

  bool regular_ready() const { return (regs_->STS_B.EOCFLG == 1); }

  uint32_t read_regular() { return regs_->REGDATA; }

  void ack_regular() {
    regs_->STS_B.REGCSFLG = 0;
    regs_->STS_B.EOCFLG = 0;
  }

public:
  void init_interrupts(uint32_t interrupt_bitset);

  static void set_interrupt_priority(IrqPriority priority) {
    set_irq_priority(ADC_IRQn, priority);
  }

  static void enable_interrupts() { enable_irq(ADC_IRQn); }

  static void disable_interrupts() { disable_irq(ADC_IRQn); }

  bool has_interrupt(ADC_INT_T interrupt) const {
    auto sts = regs_->STS_B;
    auto cr1 = regs_->CTRL1_B;

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
  static inline std::array<bool, periph_num> clk_enabled_{};
  static void enable_clk(Peripheral peripheral);
  static inline std::array<void (*)(void), periph_num> enable_clk_{
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC1); },
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC2); },
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC3); }};
};

} // namespace adc
} // namespace apm32
} // namespace mcu

#endif
#endif
