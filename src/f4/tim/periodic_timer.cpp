#include <apm32/f4/tim/driver/periodic_timer.hpp>

#include <apm32/f4/core.hpp>

#include <emb/mmio.hpp>

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

  uint32_t const period = uint32_t(timebase_freq / conf.frequency) - 1;
  core::ensure(period <= UINT16_MAX);

  emb::mmio::modify(regs.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0),   // up counting
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0),    // edge-aligned
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(0)     // div1
  );
  regs.AUTORLD = period;
  regs.PSC = conf.prescaler.value();
  regs.REPCNT = 0;
  emb::mmio::set(regs.CEG, TMR_CEG_UEG);
  emb::mmio::set(regs.CTRL1, TMR_CTRL1_ARPEN);
}

} // namespace tim
} // namespace f4
} // namespace apm32
