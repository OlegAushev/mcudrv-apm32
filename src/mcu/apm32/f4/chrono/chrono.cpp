#ifdef APM32F4XX

#include <mcu/apm32/f4/chrono.hpp>
#include <mcu/apm32/f4/system.hpp>

#include <emb/chrono.hpp>

extern "C" void SysTick_Handler() {
  mcu::apm32::chrono::steady_clock::on_interrupt();
}

std::chrono::time_point<emb::chrono::steady_clock>
emb::chrono::steady_clock::now() {
  return std::chrono::time_point<emb::chrono::steady_clock>{
      mcu::apm32::chrono::steady_clock::now().time_since_epoch()};
}

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace chrono {

void steady_clock::init() {
  // init systick
  SysTick_ConfigCLKSource(SYSTICK_CLK_SOURCE_HCLK);

  uint32_t const ticks_per_msec{core_clk_freq() / 1000};
  SysTick->LOAD = ticks_per_msec - 1;
  // set to highest priority instead of (1UL << __NVIC_PRIO_BITS) - 1UL
  NVIC_SetPriority(SysTick_IRQn, 0);
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;

  initialized_ = true;
}

void high_resolution_clock::init() {
  if (!steady_clock::initialized()) {
    fatal_error();
  }

  nsec_per_tick_ = 1'000'000'000.0f / static_cast<float>(core_clk_freq());
  initialized_ = true;
}

} //namespace chrono
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
