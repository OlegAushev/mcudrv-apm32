#pragma once

#include <apm32/device.hpp>

#include <cstdint>

namespace apm32 {
namespace f4 {
namespace core {

void init_clock();
void update_clock();
void init_core();
void reset_device();

inline uint32_t clock_freq() {
  return SystemCoreClock;
}

inline uint32_t serial_number() {
  uint32_t* uid_ptr{reinterpret_cast<uint32_t*>(0x1FFF7A10)};
  return *uid_ptr;
}

} // namespace core
} // namespace f4
} // namespace apm32
