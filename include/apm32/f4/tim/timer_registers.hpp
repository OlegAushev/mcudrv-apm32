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
  static constexpr std::array<uintptr_t, 4> ccrx =
      {base + 0x34, base + 0x38, base + 0x3C, base + 0x40};
};

} // namespace detail
} // namespace tim
} // namespace f4
} // namespace apm32
