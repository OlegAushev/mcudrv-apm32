#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcu/apm32/f4/adc.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace adc {

Module::Module(Peripheral peripheral, Config const& conf, dma::Stream* dma)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{adc::regs[std::to_underlying(peripheral_)]} {
  enable_clk(peripheral);

  if (!common_regs_) {
    common_regs_ = ADC;
    ADC_CommonConfig(const_cast<ADC_CommonConfig_T*>(&conf.hal_common_config));
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  regs_->CTRL1_B.RESSEL = conf.resolution;
  regs_->CTRL1_B.SCANEN = conf.scan_mode;

  regs_->CTRL2_B.REGEXTTRGEN = conf.ext_trigger_edge;
  regs_->CTRL2_B.REGEXTTRGSEL = conf.ext_trigger;
  regs_->CTRL2_B.DALIGNCFG = conf.data_align;
  regs_->CTRL2_B.CONTCEN = conf.continuous_mode;

  regs_->REGSEQ1_B.REGSEQLEN = conf.conv_num - 1;
#pragma GCC diagnostic pop

  if (conf.eoc_on_each_conv) {
    regs_->CTRL2_B.EOCSEL = 1;
  }

  regs_->CTRL2_B.ADCEN = 1;
  auto counter = (3 * (core_clk_freq() / 1000000));
  while (counter != 0) {
    --counter;
  }

  if (dma) {
    regs_->CTRL2_B.DMAEN = 1;
    if (conf.dma_continuous_requests) {
      regs_->CTRL2_B.DMADISSEL = 1;
    }

    write_reg(dma->stream_reg()->PADDR, uint32_t(&(regs_->REGDATA)));
  }

  if (conf.discontinuous_mode) {
    regs_->CTRL1_B.REGDISCEN = 1;
    if (conf.discontinuous_conv_num == 0 || conf.discontinuous_conv_num > 8) {
      fatal_error();
    }
    regs_->CTRL1_B.DISCNUMCFG = (conf.discontinuous_conv_num - 1) & 0x7;
  }

  // configure injected channels
  if (conf.injected.has_value()) {
    if (conf.injected.value().conv_num > 4) {
      fatal_error();
    }
    regs_->INJSEQ_B.INJSEQLEN = (conf.injected.value().conv_num - 1) & 0x3;
    regs_->CTRL1_B.INJGACEN = conf.injected.value().auto_conv;
    regs_->CTRL1_B.INJDISCEN = conf.injected.value().discontinuous_mode;
    ADC_ConfigExternalTrigInjectedConvEdge(
        regs_, conf.injected.value().ext_trigger_edge);
    ADC_ConfigExternalTrigInjectedConv(regs_,
                                       conf.injected.value().ext_trigger);
  }
}

std::unique_ptr<gpio::AnalogPin> Module::init_injected(
    PinConfig const& pinconf, InjectedChannelConfig const& chconf) {
  auto pin{std::make_unique<gpio::AnalogPin>(
      gpio::AnalogConfig{.port = pinconf.port, .pin = pinconf.pin})};

  for (auto rank : chconf.ranks) {
    ADC_ConfigInjectedChannel(regs_,
                              chconf.channel,
                              static_cast<ADC_INJEC_CHANNEL_T>(rank),
                              chconf.sampletime);
    ADC_ConfigInjectedOffset(
        regs_, static_cast<ADC_INJEC_CHANNEL_T>(rank), chconf.offset);
  }

  return pin;
}

std::unique_ptr<gpio::AnalogPin> Module::init_regular(
    PinConfig const& pinconf, RegularChannelConfig const& chconf) {
  auto pin{std::make_unique<gpio::AnalogPin>(
      gpio::AnalogConfig{.port = pinconf.port, .pin = pinconf.pin})};

  for (auto rank : chconf.ranks) {
    ADC_ConfigRegularChannel(regs_, chconf.channel, rank, chconf.sampletime);
  }

  return pin;
}

void Module::init_internal_injected(InjectedChannelConfig const& chconf) {
  for (auto rank : chconf.ranks) {
    ADC_ConfigInjectedChannel(regs_,
                              chconf.channel,
                              static_cast<ADC_INJEC_CHANNEL_T>(rank),
                              chconf.sampletime);
    ADC_ConfigInjectedOffset(
        regs_, static_cast<ADC_INJEC_CHANNEL_T>(rank), chconf.offset);
  }
  ADC_EnableTempSensorVrefint();
}

void Module::init_internal_regular(RegularChannelConfig const& chconf) {
  for (auto rank : chconf.ranks) {
    ADC_ConfigRegularChannel(regs_, chconf.channel, rank, chconf.sampletime);
  }
  ADC_EnableTempSensorVrefint();
}

void Module::init_interrupts(uint32_t interrupt_bitset) {
  regs_->STS_B.AWDFLG = 0;
  regs_->STS_B.EOCFLG = 0;
  regs_->STS_B.INJEOCFLG = 0;
  regs_->STS_B.INJCSFLG = 0;
  regs_->STS_B.REGCSFLG = 0;
  regs_->STS_B.OVREFLG = 0;

  set_bit(regs_->CTRL1, interrupt_bitset);
}

void Module::enable_clk(Peripheral peripheral) {
  auto adc_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[adc_idx]) {
    return;
  }

  enable_clk_[adc_idx]();
  clk_enabled_[adc_idx] = true;
}

} // namespace adc
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
#endif
