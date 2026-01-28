#include <apm32/f4/adc/driver/basic_adc.hpp>
#include <apm32/f4/chrono.hpp>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {
namespace detail {

void init_basic_adc(
    registers& regs,
    basic_adc_config const& conf
) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  detail::init_common();

  // Resolution = 00: 12-bit (minimum 15 ADCCLK cycles)
  regs.CTRL1_B.RESSEL = 0;

  // Scan mode = 0: Scan mode disabled
  regs.CTRL1_B.SCANEN = 1;

  // External trigger for regular channels
  if (conf.regular_trigger.has_value()) {
    regs.CTRL2_B.REGEXTTRGEN = std::to_underlying(conf.regular_trigger->edge);
    regs.CTRL2_B.REGEXTTRGSEL = std::to_underlying(conf.regular_trigger->event);
  } else {
    regs.CTRL2_B.REGEXTTRGEN = 0;
    regs.CTRL2_B.REGEXTTRGSEL = 0;
  }

  // Data alignment = 0: Right alignment
  regs.CTRL2_B.DALIGNCFG = 0;

  // Continuous conversion = 0: Single conversion mode
  regs.CTRL2_B.CONTCEN = 0;

  // Regular channel sequence length
  if (conf.regular_count > 0) {
    regs.REGSEQ1_B.REGSEQLEN = conf.regular_count - 1;
  } else {
    regs.REGSEQ1_B.REGSEQLEN = 0;
  }

  if (conf.eoc_on_each_conversion) {
    // End of conversion selection = 1:
    // The EOC bit is set at the end of each regular conversion
    regs.CTRL2_B.EOCSEL = 1;
  }

  // DMA
  if (conf.dma_enabled) {
    // DMA mode enabled
    // Direct memory access mode (for single ADC mode) = 1: DMA mode enabled
    regs.CTRL2_B.DMAEN = 1;

    // DMA disable selection (for single ADC mode) = 1:
    // DMA requests are issued as long as data are converted and DMA=1
    regs.CTRL2_B.DMADISSEL = 1;
  }

  // Injected sequence length
  regs.INJSEQ_B.INJSEQLEN = conf.injected_count;

  // Used to enable automatic conversion of injected channels after the
  // conversion of regular channel group is completed.
  regs.CTRL1_B.INJGACEN = conf.auto_injected_conversion;

  // Discontinuous mode on injected channels = 0: disabled
  regs.CTRL1_B.INJDISCEN = 0;

  // External trigger for injected channels
  if (conf.injected_trigger.has_value()) {
    regs.CTRL2_B.INJEXTTRGEN = std::to_underlying(conf.injected_trigger->edge);
    regs.CTRL2_B.INJGEXTTRGSEL = std::to_underlying(
        conf.injected_trigger->event
    );
  } else {
    regs.CTRL2_B.INJEXTTRGEN = 0;
    regs.CTRL2_B.INJGEXTTRGSEL = 0;
  }

  // Enable ADC
  regs.CTRL2_B.ADCEN = 1;
  chrono::high_resolution_clock::delay(powerup_time);

  // Interrupts configuration
  regs.STS_B.AWDFLG = 0;
  regs.STS_B.EOCFLG = 0;
  regs.STS_B.INJEOCFLG = 0;
  regs.STS_B.INJCSFLG = 0;
  regs.STS_B.REGCSFLG = 0;
  regs.STS_B.OVREFLG = 0;

  if (conf.injected_count > 0) {
    // Interrupt enable for injected channels
    regs.CTRL1_B.INJEOCIEN = 1;
  }

  if (conf.regular_count > 0 && conf.eoc_on_each_conversion) {
    // Interrupt enable for EOC
    regs.CTRL1_B.EOCIEN = 1;
  }
#pragma GCC diagnostic pop
}

} // namespace detail
} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
