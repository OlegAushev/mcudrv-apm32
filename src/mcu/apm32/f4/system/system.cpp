#ifdef APM32F4XX

#include <mcu/apm32/f4/system.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {

__attribute__((weak)) void init_clk() {
  fatal_error("device init clock func not implemented");
}

__attribute__((weak)) void update_clk() {
  fatal_error("device update clock func not implemented");
}

void init_core() {
  // assign all the interrupt priority bits to the group (preempt) priority
  // s. PM0214
  NVIC_SetPriorityGrouping(0);
}

void reset_device() {
  NVIC_SystemReset();
}

} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
