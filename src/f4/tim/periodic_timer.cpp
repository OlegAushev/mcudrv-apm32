#include <apm32/f4/tim/driver/periodic_timer.hpp>

#include <apm32/f4/core.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

void detail::configure_timebase(
    emb::units::hz_f32 clk_freq,
    registers& regs,
    periodic_timer_config const& conf
) {
  core::ensure(conf.prescaler.has_value());

  auto const timebase_freq = clk_freq /
                             static_cast<float>(conf.prescaler.value() + 1);

  TMR_BaseConfig_T base_config{};
  base_config.countMode = TMR_COUNTER_MODE_UP;
  base_config.clockDivision = TMR_CLOCK_DIV_1;
  base_config.period = static_cast<uint32_t>((timebase_freq / conf.frequency)) -
                       1;
  core::ensure(base_config.period <= UINT16_MAX);
  base_config.division = conf.prescaler.value();
  base_config.repetitionCounter = 0;

  TMR_ConfigTimeBase(&regs, &base_config);
  regs.CTRL1_B.ARPEN = 1;
}

} // namespace tim
} // namespace f4
} // namespace apm32
