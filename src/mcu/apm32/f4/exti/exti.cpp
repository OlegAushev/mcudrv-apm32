#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcu/apm32/f4/exti.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace exti {

namespace {

void enable_clk() {
  RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
}

bool clk_enabled{false};

} // namespace

void init(Line line, Mode mode, TriggerEdge edge) {
  if (!clk_enabled) {
    enable_clk();
    clk_enabled = true;
  }

  EINT_Config_T conf_{
      .line = static_cast<EINT_LINE_T>(line),
      .mode = static_cast<EINT_MODE_T>(mode),
      .trigger = static_cast<EINT_TRIGGER_T>(edge),
      .lineCmd = 1};
  EINT_Config(&conf_);
}

} // namespace exti
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
#endif
