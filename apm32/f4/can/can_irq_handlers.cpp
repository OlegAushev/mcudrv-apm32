#include <apm32/f4/can/can.hpp>

using namespace apm32::f4::can;

extern "C" void CAN1_RX0_IRQHandler() {
  if (can1::on_irq_rx0) can1::on_irq_rx0();
}

extern "C" void CAN1_RX1_IRQHandler() {
  if (can1::on_irq_rx1) can1::on_irq_rx1();
}

extern "C" void CAN1_TX_IRQHandler() {
  if (can1::on_irq_tx) can1::on_irq_tx();
}

extern "C" void CAN1_SCE_IRQHandler() {
  if (can1::on_irq_sce) can1::on_irq_sce();
}

extern "C" void CAN2_RX0_IRQHandler() {
  if (can2::on_irq_rx0) can2::on_irq_rx0();
}

extern "C" void CAN2_RX1_IRQHandler() {
  if (can2::on_irq_rx1) can2::on_irq_rx1();
}

extern "C" void CAN2_TX_IRQHandler() {
  if (can2::on_irq_tx) can2::on_irq_tx();
}

extern "C" void CAN2_SCE_IRQHandler() {
  if (can2::on_irq_sce) can2::on_irq_sce();
}
