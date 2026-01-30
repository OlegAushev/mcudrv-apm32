#pragma once

#include <cstdint>

namespace apm32 {
namespace f4 {
namespace tim {

enum class clock_division : uint32_t {
  div1 = 0b00u,
  div2 = 0b01u,
  div4 = 0b10u
};

enum class count_direction : uint32_t {
  up,
  down
};

enum class counter_mode : uint32_t {
  up,
  down,
  updown
};

} // namespace tim
} // namespace f4
} // namespace apm32
