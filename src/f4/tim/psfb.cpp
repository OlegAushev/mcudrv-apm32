#include <apm32/f4/tim/driver/psfb.hpp>

#include <apm32/f4/core.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace pwm {

void detail::configure_psfb_timebase(
    registers& REG,
    emb::units::hz_f32 clk_freq,
    psfb_pwm_config const& conf
) {
  core::ensure(conf.prescaler.has_value());

  auto const timebase_freq = clk_freq / float(conf.prescaler.value() + 1);

  uint32_t const period = uint32_t(timebase_freq / (2 * conf.frequency)) - 1;
  core::ensure(period <= UINT16_MAX);

  emb::mmio::modify(REG.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0u),
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0u),       // edge-aligned, up counting
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(std::to_underlying(conf.clkdiv))
  );
  REG.AUTORLD = period;
  REG.PSC = conf.prescaler.value();
  REG.REPCNT = 0;
  emb::mmio::set(REG.CEG, TMR_CEG_UEG);
  emb::mmio::set(REG.CTRL1, TMR_CTRL1_ARPEN);
}

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
