#include <apm32/f4/adc.hpp>

namespace apm32 {
namespace f4 {
namespace adc {

peripheral::peripheral(
    peripheral_id id,
    config const& conf,
    dma::stream* dma_stream
)
    : peripheral_type(id), id_(id), regs_(peripherals[std::to_underlying(id_)]) {
  enable_clock(id_);

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
  auto counter = (3 * (core::clock_frequency<uint32_t>() / 1000000));
  while (counter != 0) {
    --counter;
  }

  if (dma_stream) {
    regs_->CTRL2_B.DMAEN = 1;
    if (conf.dma_continuous_requests) {
      regs_->CTRL2_B.DMADISSEL = 1;
    }

    write_reg(dma_stream->regs()->PADDR, uint32_t(&(regs_->REGDATA)));
  }

  if (conf.discontinuous_mode) {
    core::ensure(
        conf.discontinuous_conv_num > 0 && conf.discontinuous_conv_num <= 8
    );
    regs_->CTRL1_B.REGDISCEN = 1;
    regs_->CTRL1_B.DISCNUMCFG = (conf.discontinuous_conv_num - 1) & 0x7;
  }

  // configure injected channels
  if (conf.injected.has_value()) {
    core::ensure(conf.injected.value().conv_num <= 4);
    regs_->INJSEQ_B.INJSEQLEN = (conf.injected.value().conv_num - 1) & 0x3;
    regs_->CTRL1_B.INJGACEN = conf.injected.value().auto_conv;
    regs_->CTRL1_B.INJDISCEN = conf.injected.value().discontinuous_mode;
    ADC_ConfigExternalTrigInjectedConvEdge(
        regs_,
        conf.injected.value().ext_trigger_edge
    );
    ADC_ConfigExternalTrigInjectedConv(regs_, conf.injected.value().ext_trigger);
  }
}

std::unique_ptr<gpio::analog_pin> peripheral::configure_injected(
    pin_config const& pinconf,
    injected_channel_config const& chconf
) {
  auto pin = std::make_unique<gpio::analog_pin>(
      gpio::analog_pin_config{.port = pinconf.port, .pin = pinconf.pin}
  );

  for (auto rank : chconf.ranks) {
    ADC_ConfigInjectedChannel(
        regs_,
        chconf.channel,
        detail::injected_selection[std::to_underlying(rank)],
        chconf.sampletime
    );
    ADC_ConfigInjectedOffset(
        regs_,
        detail::injected_selection[std::to_underlying(rank)],
        chconf.offset
    );
  }

  return pin;
}

std::unique_ptr<gpio::analog_pin> peripheral::configure_regular(
    pin_config const& pinconf,
    regular_channel_config const& chconf
) {
  auto pin = std::make_unique<gpio::analog_pin>(
      gpio::analog_pin_config{.port = pinconf.port, .pin = pinconf.pin}
  );

  for (auto rank : chconf.ranks) {
    ADC_ConfigRegularChannel(regs_, chconf.channel, rank, chconf.sampletime);
  }

  return pin;
}

void peripheral::configure_internal_injected(
    injected_channel_config const& chconf
) {
  for (auto rank : chconf.ranks) {
    ADC_ConfigInjectedChannel(
        regs_,
        chconf.channel,
        detail::injected_selection[std::to_underlying(rank)],
        chconf.sampletime
    );
    ADC_ConfigInjectedOffset(
        regs_,
        detail::injected_selection[std::to_underlying(rank)],
        chconf.offset
    );
  }
  ADC_EnableTempSensorVrefint();
}

void peripheral::configure_internal_regular(
    regular_channel_config const& chconf
) {
  for (auto rank : chconf.ranks) {
    ADC_ConfigRegularChannel(regs_, chconf.channel, rank, chconf.sampletime);
  }
  ADC_EnableTempSensorVrefint();
}

void peripheral::configure_interrupts(uint32_t interrupt_bitset) {
  regs_->STS_B.AWDFLG = 0;
  regs_->STS_B.EOCFLG = 0;
  regs_->STS_B.INJEOCFLG = 0;
  regs_->STS_B.INJCSFLG = 0;
  regs_->STS_B.REGCSFLG = 0;
  regs_->STS_B.OVREFLG = 0;

  set_bit(regs_->CTRL1, interrupt_bitset);
}

void peripheral::enable_clock(peripheral_id id) {
  auto adc_idx{std::to_underlying(id)};
  if (is_clock_enabled_[adc_idx]) {
    return;
  }

  detail::enable_clock[adc_idx]();
  is_clock_enabled_[adc_idx] = true;
}

} // namespace adc
} // namespace f4
} // namespace apm32
