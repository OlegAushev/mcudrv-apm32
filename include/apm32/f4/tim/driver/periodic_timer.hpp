#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>
#include <apm32/f4/tim/timer_utils.hpp>

#include <emb/mmio.hpp>
#include <emb/singleton.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

struct periodic_timer_config {
  emb::units::hz_f32 frequency;
  std::optional<uint16_t> prescaler;
  nvic::irq_priority irq_priority;
};

namespace detail {

void configure_timebase(
    registers& REG,
    emb::units::hz_f32 clk_freq,
    periodic_timer_config const& conf
);

} // namespace detail

template<some_general_purpose_timer Tim>
class periodic_timer : public emb::singleton<periodic_timer<Tim>> {
public:
  using timer_instance = Tim;
private:
  static inline registers& REG = timer_instance::REG;

  static constexpr nvic::irq_number const update_irqn_ =
      timer_instance::update_irqn;

  emb::units::sec_f32 period_;
public:
  periodic_timer(periodic_timer_config conf) {
    period_ = 1.f / conf.frequency;
    if (!conf.prescaler.has_value()) {
      conf.prescaler = calculate_prescaler<timer_instance>(
          conf.frequency,
          counter_mode::up
      );
    }

    timer_instance::enable_clock();

    detail::configure_timebase(
        REG,
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        conf
    );

    // Interrupt configuration
    emb::mmio::set(REG.DIEN, TMR_DIEN_UIEN);
    set_irq_priority(update_irqn_, conf.irq_priority);
  }

  emb::units::sec_f32 period() const {
    return period_;
  }

  void enable() {
    ack_update_interrupt();
    nvic::clear_pending_irq(update_irqn_);
    nvic::enable_irq(update_irqn_);
    enable_counter();
  }

  void ack_update_interrupt() {
    emb::mmio::clear_w0(REG.STS, TMR_STS_UIFLG);
  }
private:
  void enable_counter() {
    emb::mmio::set(REG.CTRL1, TMR_CTRL1_CNTEN);
  }

  void disable_counter() {
    emb::mmio::clear(REG.CTRL1, TMR_CTRL1_CNTEN);
  }
};

} // namespace tim
} // namespace f4
} // namespace apm32
