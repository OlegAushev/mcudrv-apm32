#pragma once

#include <cstdint>
#include <utility>

namespace apm32::f4::rcc {

enum class sysclk_src : std::uint32_t {
  hsi = 0b00,
  hse = 0b01,
  pll = 0b10,
};

enum class pll_src : std::uint32_t {
  hsi = 0,
  hse = 1,
};

enum class pll1c_div : std::uint32_t {
  _2 = 0b00,
  _4 = 0b01,
  _6 = 0b10,
  _8 = 0b11
};

enum class ahb_div : std::uint32_t {
  _1   = 0b0000,
  _2   = 0b1000,
  _4   = 0b1001,
  _8   = 0b1010,
  _16  = 0b1011,
  _64  = 0b1100,
  _128 = 0b1101,
  _256 = 0b1110,
  _512 = 0b1111,
};

enum class apb_div : std::uint32_t {
  _1  = 0b000,
  _2  = 0b100,
  _4  = 0b101,
  _8  = 0b110,
  _16 = 0b111,
};

constexpr std::uint32_t to_divisor(pll1c_div v) {
  switch (v) {
    case pll1c_div::_2: return 2;
    case pll1c_div::_4: return 4;
    case pll1c_div::_6: return 6;
    case pll1c_div::_8: return 8;
  }
}

constexpr std::uint32_t to_divisor(ahb_div v) {
  switch (v) {
    case ahb_div::_1:   return 1;
    case ahb_div::_2:   return 2;
    case ahb_div::_4:   return 4;
    case ahb_div::_8:   return 8;
    case ahb_div::_16:  return 16;
    case ahb_div::_64:  return 64;
    case ahb_div::_128: return 128;
    case ahb_div::_256: return 256;
    case ahb_div::_512: return 512;
  }
  std::unreachable();
}

constexpr std::uint32_t to_divisor(apb_div v) {
  switch (v) {
    case apb_div::_1:  return 1;
    case apb_div::_2:  return 2;
    case apb_div::_4:  return 4;
    case apb_div::_8:  return 8;
    case apb_div::_16: return 16;
  }
  std::unreachable();
}

} // namespace apm32::f4::rcc
