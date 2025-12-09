#include <apm32/f4/chrono.hpp>

#include <apm32/f4/core.hpp>

#include <emb/chrono.hpp>

extern "C" void SysTick_Handler() {
  apm32::f4::chrono::steady_clock::on_interrupt();
}

// emb::chrono::steady_clock::now() is intentionally defined here
// since emblib doesn't provide it's definition.
std::chrono::time_point<emb::chrono::steady_clock>
emb::chrono::steady_clock::now() {
  auto now = apm32::f4::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::time_point<emb::chrono::steady_clock>{now};
}

namespace apm32 {
namespace f4 {
namespace chrono {

void steady_clock::init() {
  core::ensure(!initialized_);

  // init systick
  SysTick_ConfigCLKSource(SYSTICK_CLK_SOURCE_HCLK);

  uint32_t const ticks_per_msec = core::clock_frequency<uint32_t>() / 1000u;
  SysTick->LOAD = ticks_per_msec - 1;
  // set to highest priority instead of (1UL << __NVIC_PRIO_BITS) - 1UL
  NVIC_SetPriority(SysTick_IRQn, 0);
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;

  initialized_ = true;
}

void high_resolution_clock::init() {
  core::ensure(steady_clock::initialized() && !initialized_);
  nsec_per_tick_ = 1'000'000'000.0f / core::clock_frequency<float>();
  initialized_ = true;
}

} //namespace chrono
} // namespace f4
} // namespace apm32
