#pragma once

#include <apm32/f4/can/can_instances.hpp>
#include <apm32/f4/can/can_utils.hpp>

#include <emb/meta.hpp>

namespace apm32 {
namespace f4 {
namespace can {

struct transceiver_config {
  //
};

template<some_can_instance Instance>
class transceiver {
public:
  using can_instance = Instance;
private:
  static inline registers& REG = can_instance::REG;
  static constexpr nvic::irq_number rx0_irqn = can_instance::rx0_irqn;
  static constexpr nvic::irq_number rx1_irqn = can_instance::rx1_irqn;
  static constexpr nvic::irq_number tx_irqn = can_instance::tx_irqn;
};

} // namespace can
} // namespace f4
} // namespace apm32
