#include <apm32/f4/tim/psfb.hpp>

#include <apm32/f4/core/core.hpp>

#include <emb/assert.hpp>
#include <emb/mmio.hpp>

#include <cstdint>

namespace apm32::f4::tim::pwm {

void detail::configure_psfb_timebase(registers& reg,
                                     emb::units::hz_f32 clk_freq,
                                     psfb_pwm_config const& conf)
{
  emb::ensure(conf.prescaler.has_value());

  auto const timebase_freq = clk_freq / float(conf.prescaler.value() + 1);

  std::uint32_t const period =
      std::uint32_t(timebase_freq / (2 * conf.frequency)) - 1;
  emb::ensure(period <= UINT16_MAX);

  emb::mmio::modify(
      reg.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0u),
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0u), // edge-aligned, up counting
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(conf.clkdiv));
  reg.AUTORLD = period;
  reg.PSC = conf.prescaler.value();
  reg.REPCNT = 0;
  emb::mmio::set<TMR_CEG_UEG>(reg.CEG);
  emb::mmio::set<TMR_CTRL1_ARPEN>(reg.CTRL1);
}

} // namespace apm32::f4::tim::pwm
