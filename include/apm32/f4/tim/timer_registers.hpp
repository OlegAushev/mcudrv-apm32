#pragma once

#include <array>
#include <cstdint>

namespace apm32 {
namespace f4 {
namespace tim {
namespace detail {

template<uint32_t Base>
struct register_addresses {
  static constexpr uint32_t base = Base;
  static constexpr std::array<uint32_t, 4> ccrx =
      {base + 0x34, base + 0x38, base + 0x3C, base + 0x40};
};

} // namespace detail
} // namespace adc
} // namespace f4
} // namespace apm32
