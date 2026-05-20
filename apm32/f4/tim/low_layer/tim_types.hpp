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

enum class capture_filter : uint32_t {
  disabled,
  div1_n2,
  div1_n4,
  div1_n8,
  div2_n6,
  div2_n8,
  div4_n6,
  div4_n8,
  div8_n6,
  div8_n8,
  div16_n5,
  div16_n6,
  div16_n8,
  div32_n5,
  div32_n6,
  div32_n8,
};

} // namespace tim
} // namespace f4
} // namespace apm32
