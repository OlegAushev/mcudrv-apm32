#include <apm32/f4/tim/half_bridge.hpp>

#include <apm32/f4/core.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace pwm {

namespace {

constexpr uint8_t get_deadtime_setup(
    emb::units::hz_f32 clk_freq,
    emb::chrono::nanoseconds_i32 const& deadtime,
    clock_division clkdiv
) {
  float const mul = static_cast<float>(1ul << std::to_underlying(clkdiv));
  float const t_dts_ns = mul * 1E9f / clk_freq.value();
  float const dt = static_cast<float>(deadtime.count());

  core::ensure(dt <= (32 + 0x1F) * 16 * t_dts_ns);

  if (dt <= 0x7F * t_dts_ns) {
    return static_cast<uint8_t>(dt / t_dts_ns) & 0x7F;
  } else if (dt <= (64 + 0x3F) * 2 * t_dts_ns) {
    return static_cast<uint8_t>((dt - 64 * 2 * t_dts_ns) / (2 * t_dts_ns)) |
           0x80;
  } else if (dt <= (32 + 0x1F) * 8 * t_dts_ns) {
    return static_cast<uint8_t>((dt - 32 * 8 * t_dts_ns) / (8 * t_dts_ns)) |
           0xC0;
  } else {
    return static_cast<uint8_t>((dt - 32 * 16 * t_dts_ns) / (16 * t_dts_ns)) |
           0xE0;
  }
}

} // namespace

void detail::configure_timebase(
    emb::units::hz_f32 clk_freq,
    registers& regs,
    base_config const& conf
) {
  core::ensure(conf.prescaler.has_value());

  auto const timebase_freq = clk_freq /
                             static_cast<float>(conf.prescaler.value() + 1);

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

[[nodiscard]] gpio::alternate_pin_config detail::get_break_input_config(
    registers& regs,
    break_pin_config const& bk_pin
) {
  gpio::alternate_pin_config pinconf = {
      .port = bk_pin.port,
      .pin = bk_pin.pin,
      .pull = bk_pin.pull,
      .output_type = gpio::output_type::pushpull,
      .speed = gpio::speed::fast,
      .altfunc = &regs == TMR1 ? GPIO_AF_TMR1 : GPIO_AF_TMR8
  };
  return pinconf;
}

void detail::configure_bdt(
    emb::units::hz_f32 clk_freq,
    registers& regs,
    base_config const& conf,
    std::optional<break_pin_config> const& bk_pin
) {
  core::ensure(conf.prescaler.has_value());

  TMR_BDTConfig_T bdt_config{};

  bdt_config.RMOS = TMR_RMOS_STATE_ENABLE;
  bdt_config.IMOS = TMR_IMOS_STATE_ENABLE;
  bdt_config.lockLevel = TMR_LOCK_LEVEL_OFF;
  bdt_config
      .deadTime = get_deadtime_setup(clk_freq, conf.deadtime, conf.clkdiv);

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

void detail::configure_channel(registers& regs, channel ch) {
  core::ensure(ch != channel::ch4);

  TMR_OCConfig_T ch_config{};
  ch_config.mode = TMR_OC_MODE_PWM1;
  ch_config.outputState = TMR_OC_STATE_ENABLE;
  ch_config.outputNState = TMR_OC_NSTATE_ENABLE;
  ch_config.polarity = TMR_OC_POLARITY_HIGH;
  ch_config.nPolarity = TMR_OC_NPOLARITY_HIGH;
  ch_config.idleState = TMR_OC_IDLE_STATE_RESET;
  ch_config.nIdleState = TMR_OC_NIDLE_STATE_RESET;
  ch_config.pulse = 0;

  switch (ch) {
  case channel::ch1:
    regs.CCM1_COMPARE_B.OC1PEN = 1;
    TMR_ConfigOC1(&regs, &ch_config);
    break;
  case channel::ch2:
    regs.CCM1_COMPARE_B.OC2PEN = 1;
    TMR_ConfigOC2(&regs, &ch_config);
    break;
  case channel::ch3:
    regs.CCM2_COMPARE_B.OC3PEN = 1;
    TMR_ConfigOC3(&regs, &ch_config);
    break;
  case channel::ch4:
    std::unreachable();
    break;
  }
}

gpio::alternate_pin_config
detail::get_output_config(registers& regs, output_pin_config const& pin) {
  GPIO_AF_T const altfunc = TMR1 ? GPIO_AF_TMR1 : GPIO_AF_TMR8;
  gpio::alternate_pin_config pinconf = {
      .port = pin.port,
      .pin = pin.pin,
      .pull = gpio::pull::none,
      .output_type = gpio::output_type::pushpull,
      .speed = gpio::speed::fast,
      .altfunc = altfunc
  };
  return pinconf;
}

static_assert(
    get_deadtime_setup(
        emb::units::hz_f32{168000000},
        std::chrono::duration<int32_t, std::nano>{500},
        clock_division::div1
    ) == 84
);

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
