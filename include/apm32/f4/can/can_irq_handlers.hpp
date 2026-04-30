#include <emb/delegate.hpp>

namespace apm32::f4::can {

using irq_handler_type = emb::delegate<void(void)>;

} // namespace apm32::f4::can

extern "C" inline void CAN1_RX0_IRQHandler() {
  //
}

extern "C" inline void CAN1_RX1_IRQHandler() {
  //
}

extern "C" inline void CAN1_TX_IRQHandler() {
  //
}

extern "C" inline void CAN2_RX0_IRQHandler() {
  //
}

extern "C" inline void CAN2_RX1_IRQHandler() {
  //
}

extern "C" inline void CAN2_TX_IRQHandler() {
  //
}
