#include <apm32/f4/chrono.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/rcc.hpp>

#include <emb/chrono.hpp>
#include <emb/mmio.hpp>
#include <emb/panic.hpp>

#include <cstdint>

extern "C" void SysTick_Handler()
{
  apm32::f4::chrono::steady_clock::on_interrupt();
}

namespace apm32::f4::chrono {

void steady_clock::init()
{
  if (initialized_) emb::panic();

  std::uint32_t const ticks_per_msec =
      rcc::hclk_frequency<std::uint32_t>() / 1000u;
  SysTick->LOAD = ticks_per_msec - 1;
  NVIC_SetPriority(SysTick_IRQn, 0);
  SysTick->VAL = 0;
  emb::mmio::set<SysTick_CTRL_CLKSOURCE_Msk
                 | SysTick_CTRL_TICKINT_Msk
                 | SysTick_CTRL_ENABLE_Msk>(SysTick->CTRL);

  initialized_ = true;
}

void high_resolution_clock::init()
{
  if (!steady_clock::initialized()) emb::panic();
  if (initialized_) emb::panic();
  nsec_per_tick_ = 1'000'000'000.0f / rcc::hclk_frequency<float>();
  initialized_ = true;
}

} // namespace apm32::f4::chrono
