#pragma once

#include <array>
#include <cstdint>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {
namespace detail {

consteval std::array<uint32_t, 4> adc_jdrx(uint32_t base) {
  return {base + 0x3C, base + 0x40, base + 0x44, base + 0x48};
}

template<uint32_t Base>
struct register_addresses {
  static constexpr uint32_t base = Base;
  static constexpr std::array<uint32_t, 4> jdrx =
      {base + 0x3C, base + 0x40, base + 0x44, base + 0x48};
};

} // namespace detail
} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
