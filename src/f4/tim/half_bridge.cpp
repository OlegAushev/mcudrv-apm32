#include <apm32/f4/tim/driver/half_bridge.hpp>

#include <apm32/f4/core.hpp>

#include <emb/mmio.hpp>

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

  uint32_t const period = static_cast<uint32_t>(
      (timebase_freq / conf.frequency) / 2
  );
  core::ensure(period <= UINT16_MAX);

  emb::mmio::modify(regs.CTRL1,
      emb::mmio::bits<TMR_CTRL1_CNTDIR>(0),
      emb::mmio::bits<TMR_CTRL1_CAMSEL>(0b11u),   // center-aligned mode 3
      emb::mmio::bits<TMR_CTRL1_CLKDIV>(std::to_underlying(conf.clkdiv))
  );
  regs.AUTORLD = period;
  regs.PSC = conf.prescaler.value();
  regs.REPCNT = 0;
  emb::mmio::set(regs.CEG, TMR_CEG_UEG);
  emb::mmio::set(regs.CTRL1, TMR_CTRL1_ARPEN);
}

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
