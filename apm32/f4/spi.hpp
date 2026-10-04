#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>
#include <apm32/f4/rcc.hpp>

#include <emb/assert.hpp>
#include <emb/meta.hpp>
#include <emb/mmio.hpp>
#include <emb/units.hpp>

#include <cstddef>
#include <cstdint>

namespace apm32::f4::spi {

using registers = SPI_TypeDef;

inline constexpr std::size_t count = 3;

inline constexpr emb::units::hz_f32 max_clock_frequency{42e6f};

struct spi1 {
  static inline registers& reg = *SPI1;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk2_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB2CLKEN_SPI1EN>(RCM->APB2CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::spi1;
};

struct spi2 {
  static inline registers& reg = *SPI2;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB1CLKEN_SPI2EN>(RCM->APB1CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::spi2;
};

struct spi3 {
  static inline registers& reg = *SPI3;

  template<typename T>
  static constexpr auto clock_frequency = rcc::pclk1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_APB1CLKEN_SPI3EN>(RCM->APB1CLKEN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::spi3;
};

template<typename T>
struct is_spi_instance
    : std::bool_constant<emb::same_as_any<T, spi1, spi2, spi3>> {};

template<typename T>
concept some_spi_instance = is_spi_instance<T>::value;

enum class error {
  timeout,
  overrun,
  underrun,
  mode_fault,
  crc_error,
  frame_error
};

struct mosi_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct miso_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct clk_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct ss_pin_config {
  gpio::port port;
  gpio::pin pin;
};

enum class clock_polarity : std::uint32_t { low = 0, high = 1 };

enum class clock_phase : std::uint32_t { first_edge = 0, second_edge = 1 };

enum class data_length : std::uint32_t { bits_8 = 0, bits_16 = 1 };

enum class bit_order : std::uint32_t { msb_first = 0, lsb_first = 1 };

enum class baudrate_prescaler : std::uint32_t {
  div2 = 0b000,
  div4 = 0b001,
  div8 = 0b010,
  div16 = 0b011,
  div32 = 0b100,
  div64 = 0b101,
  div128 = 0b110,
  div256 = 0b111,
};

template<typename T>
concept frame_format = emb::same_as_any<T, std::uint8_t, std::uint16_t>;

template<some_spi_instance Instance>
void enable()
{
  emb::mmio::set<SPI_CTRL1_SPIEN>(Instance::reg.CTRL1);
}

template<some_spi_instance Instance>
void disable()
{
  emb::mmio::clear<SPI_CTRL1_SPIEN>(Instance::reg.CTRL1);
}

inline constexpr std::array<std::uint32_t, 8> clock_prescalers =
    {2, 4, 8, 16, 32, 64, 128, 256};

namespace detail {

constexpr baudrate_prescaler calculate_prescaler(emb::units::hz_f32 clk_freq,
                                                 emb::units::hz_f32 spi_freq)
{
  std::uint32_t clk_freq_u32 = static_cast<std::uint32_t>(clk_freq.value);
  std::uint32_t spi_freq_u32 = static_cast<std::uint32_t>(spi_freq.value);

  std::uint32_t ratio =
      clk_freq_u32 / spi_freq_u32 + (clk_freq_u32 % spi_freq_u32 != 0);
  auto it =
      std::upper_bound(clock_prescalers.begin(), clock_prescalers.end(), ratio);

  emb::ensure(it != clock_prescalers.end());

  return static_cast<baudrate_prescaler>(
      std::distance(clock_prescalers.begin(), it));
}

} // namespace detail

template<some_spi_instance Instance>
baudrate_prescaler calculate_prescaler(emb::units::hz_f32 spi_freq)
{
  return detail::calculate_prescaler(
      Instance::template clock_frequency<emb::units::hz_f32>(),
      spi_freq);
}

constexpr gpio::speed pin_speed(emb::units::hz_f32 spi_freq)
{
  if (spi_freq < emb::units::hz_f32{10e6f}) {
    return gpio::speed::medium;
  }
  else if (spi_freq < emb::units::hz_f32{25e6f}) {
    return gpio::speed::very_high;
  }
  else {
    return gpio::speed::very_high;
  }
}

} // namespace apm32::f4::spi
