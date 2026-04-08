#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/meta.hpp>
#include <emb/mmio.hpp>

#include <array>
#include <cstddef>
#include <utility>

namespace apm32 {
namespace f4 {
namespace spi {

using registers = SPI_TypeDef;

inline constexpr size_t count = 3;

inline constexpr emb::units::hz_f32 max_clock_frequency{42e6f};

struct spi1 {
  static inline registers& REG = *SPI1;

  template<typename T>
  static constexpr auto clock_frequency = core::apb2_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_SPI1EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::spi1;
};

struct spi2 {
  static inline registers& REG = *SPI2;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI2EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::spi2;
};

struct spi3 {
  static inline registers& REG = *SPI3;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_SPI3EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::spi3;
};

template<typename T>
struct is_spi_instance
    : std::bool_constant<emb::same_as_any<T, spi1, spi2, spi3>> {};

template<typename T>
concept some_spi_instance = is_spi_instance<T>::value;

} // namespace spi
} // namespace f4
} // namespace apm32
