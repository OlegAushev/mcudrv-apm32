#pragma once

#include <apm32/f4/tim/timer_types.hpp>

#include <emb/singleton.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

struct periodic_config {
  emb::units::hz_f32 frequency;
  std::optional<uint16_t> prescaler;
  nvic::irq_priority irq_priority;
};

namespace detail {

void configure_timebase(
    emb::units::hz_f32 clk_freq,
    peripheral_registers* regs,
    periodic_config const& conf
);

} // namespace detail

template<general_purpose_timer Tim>
class periodic : public emb::singleton<periodic<Tim>> {
public:
  using timer = Tim;
private:
  static inline peripheral_registers* const regs_ = timer::regs;
  static inline nvic::irq_number const update_irqn_ = timer::update_irqn;

  emb::units::hz_f32 freq_;
public:
  periodic(periodic_config conf) {
    freq_ = conf.frequency;
    if (!conf.prescaler.has_value()) {
      conf.prescaler = calculate_prescaler<timer>(freq_, counter_mode::up);
    }

    timer::enable_clock();

    detail::configure_timebase(
        timer::template clock_frequency<emb::units::hz_f32>(),
        regs_,
        conf
    );

    // Interrupt configuration
    regs_->DIEN_B.UIEN = 1;
    set_irq_priority(update_irqn_, conf.irq_priority);
  }

  peripheral_registers* regs() {
    return regs_;
  }

  emb::units::hz_f32 frequency() const {
    return freq_;
  }

  void enable() {
    ack_update_interrupt();
    nvic::clear_pending_irq(update_irqn_);
    nvic::enable_irq(update_irqn_);
    enable_counter();
  }

  void ack_update_interrupt() {
    regs_->STS_B.UIFLG = 0;
  }
private:
  void enable_counter() {
    regs_->CTRL1_B.CNTEN = 1;
  }

  void disable_counter() {
    regs_->CTRL1_B.CNTEN = 0;
  }
};

} // namespace tim
} // namespace f4
} // namespace apm32
