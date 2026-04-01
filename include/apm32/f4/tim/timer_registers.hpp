#pragma once

#include <array>
#include <cstdint>

namespace apm32 {
namespace f4 {
namespace tim {
namespace detail {

template<uintptr_t Base>
struct register_addresses {
  static constexpr uintptr_t base = Base;
  static constexpr uintptr_t cnt = base + 0x24;
  static constexpr uintptr_t arr = base + 0x2C;
  static constexpr uintptr_t ccr1 = base + 0x34;
  static constexpr uintptr_t ccr2 = base + 0x38;
  static constexpr uintptr_t ccr3 = base + 0x3C;
  static constexpr uintptr_t ccr4 = base + 0x40;
  static constexpr std::array<uintptr_t, 4> ccrx = {ccr1, ccr2, ccr3, ccr4};
};

} // namespace detail
} // namespace tim
} // namespace f4
} // namespace apm32
