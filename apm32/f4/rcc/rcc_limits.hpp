#pragma once

#include <cstdint>

namespace apm32::f4::rcc {

inline constexpr std::uint64_t hse_min      = 4'000'000;
inline constexpr std::uint64_t hse_max      = 26'000'000;
inline constexpr std::uint64_t vco_in_min   = 1'000'000;
inline constexpr std::uint64_t vco_in_max   = 2'000'000;
inline constexpr std::uint64_t vco_out_min  = 100'000'000;
inline constexpr std::uint64_t vco_out_max  = 432'000'000;
inline constexpr std::uint64_t sysclk_max   = 168'000'000;
inline constexpr std::uint64_t hclk_max     = 168'000'000;
inline constexpr std::uint64_t pclk1_max    = 42'000'000;
inline constexpr std::uint64_t pclk2_max    = 84'000'000;

} // namespace apm32::f4::rcc
