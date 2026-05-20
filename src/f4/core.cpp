#include <apm32/f4/core.hpp>

#include <apm32/f4/nvic.hpp>

namespace apm32::f4::core {

__attribute__((weak)) void init_clock() {
  while (true) {
  }
}

__attribute__((weak)) void update_clock() {
  while (true) {
  }
}

void init_core() {
  // assign all the interrupt priority bits to the group (preempt) priority
  // s. PM0214
  NVIC_SetPriorityGrouping(0);
}

void reset_device() {
  NVIC_SystemReset();
}

[[noreturn]] void halt_device() {
  nvic::disable_interrupts();
  while (true) {
    __NOP();
  }
}

} // namespace apm32::f4::core
