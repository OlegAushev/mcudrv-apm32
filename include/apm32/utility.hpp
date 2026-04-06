#pragma once

#include <apm32/device.hpp>

#include <cstdint>

namespace apm32 {

enum class exec_status {
  ok,
  error,
  busy,
  timeout,
  invalid_argument,
  overflow,
};

inline uint32_t bit_position(uint32_t val) {
  return __CLZ(__RBIT(val));
}

} // namespace apm32
