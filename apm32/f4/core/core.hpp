#pragma once

#include <apm32/device.hpp>

#include <cstdint>

namespace apm32::f4::core {

void init_core();
void reset_device();
[[noreturn]] void halt_device();

inline std::uint32_t serial_number() {
  std::uint32_t* uid_ptr{reinterpret_cast<std::uint32_t*>(0x1FFF7A10)};
  return *uid_ptr;
}

} // namespace apm32::f4::core
