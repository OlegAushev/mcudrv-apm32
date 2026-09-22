#include <apm32/f4/tim/hall_interface.hpp>

#include <emb/mmio.hpp>

namespace apm32::f4::tim::hall {

void detail::configure_timebase(registers& reg,
                                detail::timebase_config const& cfg)
{
  emb::mmio::modify(
      reg.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0u),
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0u),
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(cfg.filter_clock_division));
  reg.AUTORLD = cfg.counter_max;
  reg.PSC = cfg.counter_prescaler;

  emb::mmio::set<TMR_CEG_UEG>(reg.CEG);

  // TI1SEL = 1: XOR of CH1, CH2, CH3 inputs
  emb::mmio::set<TMR_CTRL2_TI1SEL>(reg.CTRL2);

  // Slave mode: trigger = TI1 Edge Detector, mode = Reset
  emb::mmio::modify(reg.SMCTRL,
                    emb::mmio::bits<TMR_SMCTRL_TRGSEL>(0b100u), // TI1F_ED
                    emb::mmio::bits<TMR_SMCTRL_SMFSEL>(0b100u)  // Reset Mode
  );
}

void detail::configure_channel(registers& reg, capture_filter filter)
{
  // IC1: both edges, direct TI, prescaler = 1
  emb::mmio::modify(reg.CCM1,
                    emb::mmio::bits<TMR_CCM1_CC1SEL>(0b01u), // direct TI
                    emb::mmio::bits<TMR_CCM1_IC1F>(filter));
  // polarity = both edges: CC1POL=1, CC1NPOL=1
  emb::mmio::modify(reg.CCEN,
                    emb::mmio::bits<TMR_CCEN_CC1EN>(1u),
                    emb::mmio::bits<TMR_CCEN_CC1POL>(1u),
                    emb::mmio::bits<TMR_CCEN_CC1NPOL>(1u));
}

} // namespace apm32::f4::tim::hall
