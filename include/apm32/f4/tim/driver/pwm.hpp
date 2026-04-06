#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>

#include <emb/chrono.hpp>
#include <emb/mmio.hpp>
#include <emb/units.hpp>

namespace apm32 {
namespace f4 {
namespace tim {
namespace pwm {

struct output_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct break_pin_config {
  gpio::port port;
  gpio::pin pin;
  gpio::pull pull;
  emb::gpio::level active_level;
};

enum class trigger_output {
  none,
  update
};

namespace detail {

template<advanced_timer Tim>
[[nodiscard]] gpio::alternate_pin_config
make_output_gpio_config(output_pin_config const& pin) {
  return gpio::alternate_pin_config{
      .port = pin.port,
      .pin = pin.pin,
      .pull = gpio::pull::none,
      .output_type = gpio::output_type::pushpull,
      .speed = gpio::speed::medium,
      .altfunc = Tim::gpio_altfunc
  };
}

template<advanced_timer Tim>
[[nodiscard]] gpio::alternate_pin_config
make_break_input_gpio_config(break_pin_config const& bk_pin) {
  return gpio::alternate_pin_config{
      .port = bk_pin.port,
      .pin = bk_pin.pin,
      .pull = bk_pin.pull,
      .output_type = gpio::output_type::pushpull,
      .speed = gpio::speed::low,
      .altfunc = Tim::gpio_altfunc
  };
}

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

void configure_bdt(
    registers& regs,
    emb::units::hz_f32 clk_freq,
    emb::chrono::nanoseconds_i32 const& deadtime,
    clock_division clkdiv,
    std::optional<break_pin_config> const& bk_pin
);

} // namespace detail

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
