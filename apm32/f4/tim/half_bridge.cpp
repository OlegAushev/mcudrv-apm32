#include <apm32/f4/tim/half_bridge.hpp>

#include <apm32/f4/core/core.hpp>

#include <emb/assert.hpp>
#include <emb/mmio.hpp>

#include <cstdint>

namespace apm32::f4::tim::pwm {

void detail::configure_half_bridge_timebase(registers& reg,
                                            emb::units::hz_f32 clk_freq,
                                            half_bridge_pwm_config const& conf)
{
  emb::ensure(conf.prescaler.has_value());

  auto const timebase_freq =
      clk_freq / static_cast<float>(conf.prescaler.value() + 1);

  std::uint32_t const period =
      static_cast<std::uint32_t>((timebase_freq / conf.frequency) / 2);
  emb::ensure(period <= UINT16_MAX);

  emb::mmio::modify(
      reg.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0),
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0b11u), // center-aligned mode 3
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(conf.clkdiv));
  reg.AUTORLD = period;
  reg.PSC = conf.prescaler.value();
  reg.REPCNT = 0;
  emb::mmio::set<TMR_CEG_UEG>(reg.CEG);
  emb::mmio::set<TMR_CTRL1_ARPEN>(reg.CTRL1);
}

} // namespace apm32::f4::tim::pwm
