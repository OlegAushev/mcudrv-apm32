#include <apm32/f4/tim/driver/hall_interface.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace hall {

void detail::configure_timebase(
    registers& regs,
    detail::timebase_config const& cfg
) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  regs.CTRL1_B.CNTDIR = 0;
  regs.CTRL1_B.CAMSEL = 0;
  regs.CTRL1_B.CLKDIV = std::to_underlying(cfg.filter_clock_division);
  regs.AUTORLD = cfg.counter_max;
  regs.PSC = cfg.counter_prescaler;

  regs.CEG_B.UEG = 1;

  regs.CTRL2_B.TI1SEL = 1;

  regs.SMCTRL_B.TRGSEL = 0b100u; // TI1 Edge Detector (TI1F_ED)
  regs.SMCTRL_B.SMFSEL = 0b100u; // Reset Mode
#pragma GCC diagnostic pop
}

void detail::configure_channel(registers& regs, capture_filter filter) {
  TMR_ICConfig_T channel_cfg{
      .channel = TMR_CHANNEL_1,
      .polarity = TMR_IC_POLARITY_BOTHEDGE,
      .selection = TMR_IC_SELECTION_DIRECT_TI,
      .prescaler = TMR_IC_PSC_1,
      .filter = static_cast<uint16_t>(filter)
  };
  TMR_ConfigIC(&regs, &channel_cfg);
}

} // namespace hall
} // namespace tim
} // namespace f4
} // namespace apm32
