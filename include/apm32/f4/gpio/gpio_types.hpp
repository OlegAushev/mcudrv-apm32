#pragma once

#include <apm32/device.hpp>

#include <cstddef>

namespace apm32 {
namespace f4 {
namespace gpio {
namespace v2 {

using port_registers = GPIO_TypeDef;

inline constexpr size_t port_count = 9;

} // namespace v2
} // namespace gpio
} // namespace f4
} // namespace apm32
