#include <apm32/f4/tim/driver/hall_interface.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace hall {

void detail::configure_timebase(
    registers& REG,
    detail::timebase_config const& cfg
) {
  emb::mmio::modify(REG.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0u),
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0u),
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(std::to_underlying(cfg.filter_clock_division))
  );
  REG.AUTORLD = cfg.counter_max;
  REG.PSC = cfg.counter_prescaler;

  emb::mmio::set(REG.CEG, TMR_CEG_UEG);

  // TI1SEL = 1: XOR of CH1, CH2, CH3 inputs
  emb::mmio::set(REG.CTRL2, TMR_CTRL2_TI1SEL);

  // Slave mode: trigger = TI1 Edge Detector, mode = Reset
  emb::mmio::modify(REG.SMCTRL,
      emb::mmio::bits<TMR_SMCTRL_TRGSEL>(0b100u),   // TI1F_ED
      emb::mmio::bits<TMR_SMCTRL_SMFSEL>(0b100u)    // Reset Mode
  );
}

void detail::configure_channel(registers& REG, capture_filter filter) {
  // IC1: both edges, direct TI, prescaler = 1
  emb::mmio::modify(REG.CCM1,
      emb::mmio::bits<TMR_CCM1_CC1SEL>(0b01u),      // direct TI
      emb::mmio::bits<TMR_CCM1_IC1F>(std::to_underlying(filter))
  );
  // polarity = both edges: CC1POL=1, CC1NPOL=1
  emb::mmio::modify(REG.CCEN,
      emb::mmio::bits<TMR_CCEN_CC1EN>(1u),
      emb::mmio::bits<TMR_CCEN_CC1POL>(1u),
      emb::mmio::bits<TMR_CCEN_CC1NPOL>(1u)
  );
}

} // namespace hall
} // namespace tim
} // namespace f4
} // namespace apm32
