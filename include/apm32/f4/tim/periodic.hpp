#pragma once

#include <apm32/f4/tim/timer_types.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

struct periodic_config {
  emb::units::hz_f32 frequency;
  std::optional<uint16_t> prescaler;
  nvic::irq_priority irq_priority;
};

template<general_purpose_timer Tim>
class periodic {
public:
  using timer = Tim;
private:

};

} // namespace tim
} // namespace f4
} // namespace apm32
