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

template<typename T>
bool bit_is_set(T const volatile& reg, T bit) {
  return (reg & bit) == bit;
}

template<typename T>
bool bit_is_clear(T const volatile& reg, T bit) {
  return (reg & bit) == 0;
}

template<typename T>
void set_bit(T volatile& reg, T bit) {
  reg |= bit;
}

template<typename T>
void clear_bit(T volatile& reg, T bit) {
  reg &= ~bit;
}

template<typename T>
T read_bit(T const volatile& reg, T bit) {
  return reg & bit;
}

template<typename T>
void clear_reg(T volatile& reg) {
  reg = 0;
}

template<typename T>
void write_reg(T volatile& reg, T val) {
  reg = val;
}

template<typename T>
T read_reg(T const volatile& reg) {
  return reg;
}

template<typename T>
void modify_reg(T volatile& reg, T clearmask, T setmask) {
  reg = (reg & ~clearmask) | setmask;
}

inline uint32_t bit_position(uint32_t val) {
  return __CLZ(__RBIT(val));
}

} // namespace apm32
