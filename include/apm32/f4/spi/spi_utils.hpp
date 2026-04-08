#pragma once

#include <apm32/f4/spi/spi_instances.hpp>
#include <apm32/f4/spi/spi_types.hpp>

#include <emb/mmio.hpp>
#include <emb/units.hpp>

namespace apm32 {
namespace f4 {
namespace spi {

template<some_spi_instance Instance>
void enable() {
  registers& regs = Instance::regs;
  emb::mmio::set(regs.CTRL1, SPI_CTRL1_SPIEN);
}

template<some_spi_instance Instance>
void disable() {
  registers& regs = Instance::regs;
  emb::mmio::clear(regs.CTRL1, SPI_CTRL1_SPIEN);
}

inline constexpr std::array<uint32_t, 8> clock_prescalers =
    {2, 4, 8, 16, 32, 64, 128, 256};

namespace detail {

constexpr baudrate_prescaler
calculate_prescaler(emb::units::hz_f32 clk_freq, emb::units::hz_f32 spi_freq) {
  uint32_t clk_freq_u32 = static_cast<uint32_t>(clk_freq.value());
  uint32_t spi_freq_u32 = static_cast<uint32_t>(spi_freq.value());

  uint32_t ratio = clk_freq_u32 / spi_freq_u32 +
                   (clk_freq_u32 % spi_freq_u32 != 0);
  auto it = std::upper_bound(
      clock_prescalers.begin(),
      clock_prescalers.end(),
      ratio
  );

  core::ensure(it != clock_prescalers.end());

  return static_cast<baudrate_prescaler>(
      std::distance(clock_prescalers.begin(), it)
  );
}

} // namespace detail

template<some_spi_instance Instance>
baudrate_prescaler calculate_prescaler(emb::units::hz_f32 spi_freq) {
  return detail::calculate_prescaler(Instance::clock_frequency(), spi_freq);
}

constexpr gpio::speed pin_speed(emb::units::hz_f32 spi_freq) {
  if (spi_freq < emb::units::hz_f32{10e6f}) {
    return gpio::speed::medium;
  } else if (spi_freq < emb::units::hz_f32{25e6f}) {
    return gpio::speed::very_high;
  } else {
    return gpio::speed::very_high;
  }
}

} // namespace spi
} // namespace f4
} // namespace apm32
