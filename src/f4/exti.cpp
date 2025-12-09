#include <apm32/f4/exti.hpp>

#include <apm32f4xx_rcm.h>

namespace apm32 {
namespace f4 {
namespace exti {

namespace {

void enable_clock() {
  RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
}

bool clock_enabled = false;

} // namespace

void configure(line l, mode m, trigger_edge edge) {
  if (!clock_enabled) {
    enable_clock();
    clock_enabled = true;
  }

  EINT_Config_T conf_ = {
      .line = static_cast<EINT_LINE_T>(l),
      .mode = static_cast<EINT_MODE_T>(m),
      .trigger = static_cast<EINT_TRIGGER_T>(edge),
      .lineCmd = 1
  };
  EINT_Config(&conf_);
}

} // namespace exti
} // namespace f4
} // namespace apm32
