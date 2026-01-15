#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32f4xx_gpio.h>
#include <apm32f4xx_rcm.h>

#include <cstddef>

namespace apm32 {
namespace f4 {
namespace gpio {
namespace v2 {

using port_registers = GPIO_T;

inline constexpr size_t port_count = 9;

} // namespace v2
} // namespace gpio
} // namespace f4
} // namespace apm32
