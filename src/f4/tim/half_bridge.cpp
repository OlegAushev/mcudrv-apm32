#include <apm32/f4/tim/driver/half_bridge.hpp>

#include <apm32/f4/core.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace pwm {

void detail::configure_half_bridge_timebase(
    registers& regs,
    emb::units::hz_f32 clk_freq,
    half_bridge_pwm_config const& conf
) {
  core::ensure(conf.prescaler.has_value());

  auto const timebase_freq = clk_freq
                           / static_cast<float>(conf.prescaler.value() + 1);

  TMR_BaseConfig_T base_config{};
  base_config.countMode = TMR_COUNTER_MODE_CENTER_ALIGNED3;
  base_config.clockDivision = tim::detail::to_sdk(conf.clkdiv);
  base_config.period = static_cast<uint32_t>(
      (timebase_freq / conf.frequency) / 2
  );
  core::ensure(base_config.period <= UINT16_MAX);
  base_config.division = conf.prescaler.value();
  base_config.repetitionCounter = 0;

  TMR_ConfigTimeBase(&regs, &base_config);
  regs.CTRL1_B.ARPEN = 1;
}

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
