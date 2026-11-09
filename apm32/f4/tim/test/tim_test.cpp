#include <apm32/f4/tim.hpp>
#include <apm32/f4/tim/pwm.hpp>

#include <cstdint>

using namespace apm32::f4::tim;

namespace {

static_assert(detail::calculate_prescaler<tim1>(emb::units::hz_f32{168000000},
                                                emb::units::hz_f32{10000},
                                                counter_mode::updown)
              == 0);

static_assert(detail::calculate_prescaler<tim1>(emb::units::hz_f32{168000000},
                                                emb::units::hz_f32{1000},
                                                counter_mode::updown)
              == 1);

static_assert(pwm::detail::get_deadtime_setup(
                  emb::units::hz_f32{168000000},
                  std::chrono::duration<std::int32_t, std::nano>{500},
                  clock_division::div1)
              == 84);

constexpr std::uint8_t
deadtime_setup(std::int32_t ns, clock_division clkdiv = clock_division::div1)
{
  return pwm::detail::get_deadtime_setup(emb::units::hz_f32{168000000},
                                         emb::chrono::nanoseconds_i32{ns},
                                         clkdiv);
}

static_assert(deadtime_setup(0) == 0x00);
static_assert(deadtime_setup(1) == 0x01);
static_assert(deadtime_setup(200) == 0x22);
static_assert(deadtime_setup(755) == 0x7F);

static_assert(deadtime_setup(756) == 0x80);
static_assert(deadtime_setup(762) == 0x81);
static_assert(deadtime_setup(1500) == 0xBE);
static_assert(deadtime_setup(1511) == 0xBF);

static_assert(deadtime_setup(1512) == 0xC0);
static_assert(deadtime_setup(1524) == 0xC1);
static_assert(deadtime_setup(3000) == 0xDF);

static_assert(deadtime_setup(3001) == 0xE0);
static_assert(deadtime_setup(3048) == 0xE1);
static_assert(deadtime_setup(6000) == 0xFF);

static_assert(deadtime_setup(1000, clock_division::div2) == 0x54);
static_assert(deadtime_setup(2000, clock_division::div4) == 0x54);

} // namespace
