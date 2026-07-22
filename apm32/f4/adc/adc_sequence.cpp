#include <apm32/f4/adc/adc_sequence.hpp>
#include <apm32/f4/adc/common_adc.hpp>
#include <apm32/f4/chrono/chrono.hpp>

#include <emb/mmio.hpp>

#include <cstdint>

namespace apm32::f4::adc::detail {

void init_sequence(registers& REG, sequence_config const& conf) {
  detail::init_common();

  // Resolution = 00: 12-bit
  // Scan mode enabled
  emb::mmio::modify(
      REG.CTRL1,
      emb::mmio::bits<ADC_CTRL1_RESSEL>(0u),
      emb::mmio::bits<ADC_CTRL1_SCANEN>(1u),
      emb::mmio::bits<ADC_CTRL1_INJGACEN>(
          conf.auto_injected_conversion ? 1u : 0u
      ),
      emb::mmio::bits<ADC_CTRL1_INJDISCEN>(0u)
  );

  // External trigger for regular channels
  std::uint32_t reg_ext_trgen = 0;
  std::uint32_t reg_ext_trgsel = 0;
  if (conf.regular_trigger.has_value()) {
    reg_ext_trgen = std::to_underlying(conf.regular_trigger->edge);
    reg_ext_trgsel = std::to_underlying(conf.regular_trigger->event);
  }

  // External trigger for injected channels
  std::uint32_t inj_ext_trgen = 0;
  std::uint32_t inj_ext_trgsel = 0;
  if (conf.injected_trigger.has_value()) {
    inj_ext_trgen = std::to_underlying(conf.injected_trigger->edge);
    inj_ext_trgsel = std::to_underlying(conf.injected_trigger->event);
  }

  emb::mmio::modify(
      REG.CTRL2,
      emb::mmio::bits<ADC_CTRL2_REGEXTTRGEN>(reg_ext_trgen),
      emb::mmio::bits<ADC_CTRL2_REGEXTTRGSEL>(reg_ext_trgsel),
      emb::mmio::bits<ADC_CTRL2_DALIGNCFG>(0u), // right alignment
      emb::mmio::bits<ADC_CTRL2_CONTCEN>(0u),   // single conversion
      emb::mmio::bits<ADC_CTRL2_EOCSEL>(conf.eoc_on_each_conversion ? 1u : 0u),
      emb::mmio::bits<ADC_CTRL2_DMAEN>(conf.dma_enabled ? 1u : 0u),
      emb::mmio::bits<ADC_CTRL2_DMADISSEL>(conf.dma_enabled ? 1u : 0u),
      emb::mmio::bits<ADC_CTRL2_INJEXTTRGEN>(inj_ext_trgen),
      emb::mmio::bits<ADC_CTRL2_INJGEXTTRGSEL>(inj_ext_trgsel)
  );

  // Regular channel sequence length
  if (conf.regular_count > 0) {
    emb::mmio::write<ADC_REGSEQ1_REGSEQLEN>(
        REG.REGSEQ1,
        conf.regular_count - 1
    );
  } else {
    emb::mmio::write<ADC_REGSEQ1_REGSEQLEN>(REG.REGSEQ1, 0u);
  }

  // Injected sequence length (INJSEQLEN = number of conversions - 1)
  if (conf.injected_count > 0) {
    emb::mmio::write<ADC_INJSEQ_INJSEQLEN>(REG.INJSEQ, conf.injected_count - 1);
  } else {
    emb::mmio::write<ADC_INJSEQ_INJSEQLEN>(REG.INJSEQ, 0u);
  }

  // Enable ADC
  emb::mmio::set(REG.CTRL2, ADC_CTRL2_ADCEN);
  chrono::high_resolution_clock::delay(powerup_time);

  // Clear status flags
  REG.STS = 0;

  // Interrupts configuration
  if (conf.injected_count > 0) {
    emb::mmio::set(REG.CTRL1, ADC_CTRL1_INJEOCIEN);
  }

  if (conf.regular_count > 0 && conf.eoc_on_each_conversion) {
    emb::mmio::set(REG.CTRL1, ADC_CTRL1_EOCIEN);
  }
}

} // namespace apm32::f4::adc::detail
