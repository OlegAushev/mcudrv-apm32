#pragma once

#include <apm32/device.hpp>

#include <cstddef>

namespace apm32::f4::gpio::v2 {

using port_registers = GPIO_TypeDef;

inline constexpr size_t port_count = 9;

} // namespace apm32::f4::gpio::v2
