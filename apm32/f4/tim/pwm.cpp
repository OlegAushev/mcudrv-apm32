#include <apm32/f4/tim/pwm.hpp>

#include <apm32/f4/core/core.hpp>

#include <emb/mmio.hpp>

#include <cstdint>

namespace apm32::f4::tim::pwm {

void detail::configure_bdt(
    registers& REG,
    emb::units::hz_f32 clk_freq,
    emb::chrono::nanoseconds_i32 const& deadtime,
    clock_division clkdiv,
    std::optional<break_pin_config> const& bk_pin
) {
  std::uint32_t brk_enable = 0;
  std::uint32_t brk_polarity = 0;

  if (bk_pin.has_value()) {
    brk_enable = 1;
    brk_polarity = bk_pin->polarity == emb::gpio::polarity::active_low ? 0u
                                                                       : 1u;
  }

  emb::mmio::modify(REG.BDT,
      emb::mmio::bits<TMR_BDT_RMOS>(1u),
      emb::mmio::bits<TMR_BDT_IMOS>(1u),
      emb::mmio::bits<TMR_BDT_LOCKCFG>(0u),
      emb::mmio::bits<TMR_BDT_DTS>(get_deadtime_setup(clk_freq, deadtime, clkdiv)),
      emb::mmio::bits<TMR_BDT_BRKEN>(brk_enable),
      emb::mmio::bits<TMR_BDT_BRKPOL>(brk_polarity),
      emb::mmio::bits<TMR_BDT_AOEN>(0u)
  );
}

} // namespace apm32::f4::tim::pwm
