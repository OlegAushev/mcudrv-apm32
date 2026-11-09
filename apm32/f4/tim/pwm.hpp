#pragma once

#include <apm32/f4/tim.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio/alternate_pin.hpp>

#include <emb/chrono.hpp>
#include <emb/mmio.hpp>
#include <emb/panic.hpp>
#include <emb/units.hpp>

#include <cstdint>

namespace apm32::f4::tim::pwm {

struct output_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct break_pin_config {
  gpio::port port;
  gpio::pin pin;
  gpio::pull pull;
  emb::gpio::polarity polarity;
};

namespace detail {

template<advanced_timer Tim>
[[nodiscard]] gpio::alternate_pin_config
make_output_gpio_config(output_pin_config const& pin)
{
  return gpio::alternate_pin_config{.port = pin.port,
                                    .pin = pin.pin,
                                    .pull = gpio::pull::none,
                                    .output_type = gpio::output_type::pushpull,
                                    .speed = gpio::speed::medium,
                                    .altfunc = Tim::gpio_altfunc};
}

template<advanced_timer Tim>
[[nodiscard]] gpio::alternate_pin_config
make_break_input_gpio_config(break_pin_config const& bk_pin)
{
  return gpio::alternate_pin_config{.port = bk_pin.port,
                                    .pin = bk_pin.pin,
                                    .pull = bk_pin.pull,
                                    .output_type = gpio::output_type::pushpull,
                                    .speed = gpio::speed::low,
                                    .altfunc = Tim::gpio_altfunc};
}

constexpr std::uint8_t
get_deadtime_setup(emb::units::hz_f32 clk_freq,
                   emb::chrono::nanoseconds_i32 const& deadtime,
                   clock_division clkdiv)
{
  if (deadtime.count() < 0) emb::panic();

  auto const ns = static_cast<std::uint64_t>(deadtime.count());
  std::uint64_t const clk_hz = static_cast<std::uint32_t>(clk_freq.value);
  std::uint64_t const divisor = 1'000'000'000ull << std::to_underlying(clkdiv);
  std::uint64_t const ticks = (ns * clk_hz + divisor - 1u) / divisor;

  if (ticks > 1008u) emb::panic();

  if (ticks <= 127u) return std::uint8_t(ticks);
  if (ticks <= 254u) return std::uint8_t(((ticks + 1u) / 2u - 64u) | 0x80u);
  if (ticks <= 504u) return std::uint8_t(((ticks + 7u) / 8u - 32u) | 0xC0u);
  return std::uint8_t(((ticks + 15u) / 16u - 32u) | 0xE0u);
}

void configure_bdt(registers& reg,
                   emb::units::hz_f32 clk_freq,
                   emb::chrono::nanoseconds_i32 const& deadtime,
                   clock_division clkdiv,
                   std::optional<break_pin_config> const& bk_pin);

} // namespace detail

} // namespace apm32::f4::tim::pwm
