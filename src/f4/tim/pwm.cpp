#include <apm32/f4/tim/driver/pwm.hpp>

#include <apm32/f4/core.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace pwm {

void detail::configure_bdt(
    registers& regs,
    emb::units::hz_f32 clk_freq,
    emb::chrono::nanoseconds_i32 const& deadtime,
    clock_division clkdiv,
    std::optional<break_pin_config> const& bk_pin
) {
  TMR_BDTConfig_T bdt_config{};

  bdt_config.RMOS = TMR_RMOS_STATE_ENABLE;
  bdt_config.IMOS = TMR_IMOS_STATE_ENABLE;
  bdt_config.lockLevel = TMR_LOCK_LEVEL_OFF;
  bdt_config.deadTime = get_deadtime_setup(clk_freq, deadtime, clkdiv);

  if (bk_pin.has_value()) {
    bdt_config.BRKState = TMR_BRK_STATE_ENABLE;
    bdt_config.BRKPolarity = bk_pin->active_level == emb::gpio::level::low ?
                                 TMR_BRK_POLARITY_LOW :
                                 TMR_BRK_POLARITY_HIGH;
  } else {
    bdt_config.BRKState = TMR_BRK_STATE_DISABLE;
    bdt_config.BRKPolarity = TMR_BRK_POLARITY_LOW;
  }

  bdt_config.automaticOutput = TMR_AUTOMATIC_OUTPUT_DISABLE;

  TMR_ConfigBDT(&regs, &bdt_config);
}

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
