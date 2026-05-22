#pragma once

#include <apm32/device.hpp>

#include <array>
#include <cstdint>

namespace apm32::f4::gpio {

using port_registers = GPIO_TypeDef;

inline constexpr std::size_t port_count = 9;

inline std::array<port_registers*, port_count> const ports =
    {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

enum class port : uint32_t {
  gpioa,
  gpiob,
  gpioc,
  gpiod,
  gpioe,
  gpiof,
  gpiog,
  gpioh,
  gpioi
};

} // namespace apm32::f4::gpio
