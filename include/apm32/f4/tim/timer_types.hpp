#pragma once

#include <cstdint>

namespace apm32 {
namespace f4 {
namespace tim {

enum class channel : uint32_t {
  ch1,
  ch2,
  ch3,
  ch4
};

enum class clock_division : uint32_t {
  div1,
  div2,
  div4
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
