#include <apm32/f4/tim/driver/hall_interface.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace hall {

void detail::configure_timebase(
    registers& regs,
    hall_interface_config const& conf
) {
  core::ensure(conf.prescaler.has_value() && conf.period.has_value());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  regs.CTRL1_B.CNTDIR = 0;
  regs.CTRL1_B.CAMSEL = 0;
  regs.CTRL1_B.CLKDIV = std::to_underlying(conf.clkdiv);
  regs.AUTORLD = conf.period.value();
  regs.PSC = conf.prescaler.value();

  regs.CEG_B.UEG = 1;

  regs.CTRL2_B.TI1SEL = 1;
#pragma GCC diagnostic pop
}

void detail::configure_channel(registers& regs) {
  TMR_ICConfig_T ch_config{
      .channel = TMR_CHANNEL_1,
      .polarity = TMR_IC_POLARITY_BOTHEDGE,
      .selection = TMR_IC_SELECTION_DIRECT_TI,
      .prescaler = TMR_IC_PSC_1,
      .filter = 0
  };
  TMR_ConfigIC(&regs, &ch_config);
}

} // namespace hall
} // namespace tim
} // namespace f4
} // namespace apm32
