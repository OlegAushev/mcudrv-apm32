#include <apm32/f4/exti/exti.hpp>

#include <emb/mmio.hpp>

#include <cstdint>

namespace apm32::f4::exti {

namespace {

void enable_clock() {
  emb::mmio::set<RCM_APB2CLKEN_SYSCFGEN>(RCM->APB2CLKEN);
}

bool clock_enabled = false;

} // namespace

void configure(line l, mode m, trigger_edge edge) {
  if (!clock_enabled) {
    enable_clock();
    clock_enabled = true;
  }

  std::uint32_t const line_mask = std::to_underlying(l);

  // set interrupt or event mask
  if (m == mode::interrupt) {
    emb::mmio::runtime::set(EINT->IMASK, line_mask);
  } else {
    emb::mmio::runtime::set(EINT->EMASK, line_mask);
  }

  // set trigger edge
  if (edge == trigger_edge::rising || edge == trigger_edge::both) {
    emb::mmio::runtime::set(EINT->RTEN, line_mask);
  } else {
    emb::mmio::runtime::clear(EINT->RTEN, line_mask);
  }

  if (edge == trigger_edge::falling || edge == trigger_edge::both) {
    emb::mmio::runtime::set(EINT->FTEN, line_mask);
  } else {
    emb::mmio::runtime::clear(EINT->FTEN, line_mask);
  }
}

} // namespace apm32::f4::exti
