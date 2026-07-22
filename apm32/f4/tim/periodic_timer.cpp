#include <apm32/f4/tim/periodic_timer.hpp>

#include <apm32/f4/core/core.hpp>

#include <emb/assert.hpp>
#include <emb/mmio.hpp>

#include <cstdint>

namespace apm32::f4::tim {

void detail::configure_timebase(
    registers& REG,
    emb::units::hz_f32 clk_freq,
    periodic_timer_config const& conf
) {
  emb::ensure(conf.prescaler.has_value());

  auto const timebase_freq = clk_freq /
                             static_cast<float>(conf.prescaler.value() + 1);

  std::uint32_t const period = std::uint32_t(timebase_freq / conf.frequency)
                             - 1;
  emb::ensure(period <= UINT16_MAX);

  emb::mmio::modify(REG.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0),   // up counting
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0),    // edge-aligned
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(0)     // div1
  );
  REG.AUTORLD = period;
  REG.PSC = conf.prescaler.value();
  REG.REPCNT = 0;
  emb::mmio::set<TMR_CEG_UEG>(REG.CEG);
  emb::mmio::set<TMR_CTRL1_ARPEN>(REG.CTRL1);
}

} // namespace apm32::f4::tim
