#include <apm32/f4/exti.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace exti {

namespace {

void enable_clock() {
  emb::mmio::set(RCM->APB2CLKEN, RCM_APB2CLKEN_SYSCFGEN);
}

bool clock_enabled = false;

} // namespace

void configure(line l, mode m, trigger_edge edge) {
  if (!clock_enabled) {
    enable_clock();
    clock_enabled = true;
  }

  uint32_t const line_mask = std::to_underlying(l);

  // set interrupt or event mask
  if (m == mode::interrupt) {
    emb::mmio::set(EINT->IMASK, line_mask);
  } else {
    emb::mmio::set(EINT->EMASK, line_mask);
  }

  // set trigger edge
  if (edge == trigger_edge::rising || edge == trigger_edge::both) {
    emb::mmio::set(EINT->RTEN, line_mask);
  } else {
    emb::mmio::clear(EINT->RTEN, line_mask);
  }

  if (edge == trigger_edge::falling || edge == trigger_edge::both) {
    emb::mmio::set(EINT->FTEN, line_mask);
  } else {
    emb::mmio::clear(EINT->FTEN, line_mask);
  }
}

} // namespace exti
} // namespace f4
} // namespace apm32
