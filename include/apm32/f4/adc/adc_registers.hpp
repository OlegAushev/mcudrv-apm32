#pragma once

#include <array>
#include <cstdint>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {
namespace detail {

template<uintptr_t Base>
struct register_addresses {
  static constexpr uintptr_t base = Base;
  static constexpr std::array<uintptr_t, 4> jdrx =
      {base + 0x3C, base + 0x40, base + 0x44, base + 0x48};
};

} // namespace detail
} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
