#pragma once

#include <apm32/f4/tim/tim.hpp>

#include <emb/mmio.hpp>

#include <cstdint>
#include <utility>

namespace apm32::f4::tim {

struct periodic_timer_config {
  emb::units::hz_f32 frequency;
  std::optional<std::uint16_t> prescaler;
  nvic::irq_priority irq_priority;
  std::optional<trigger_output> trgo;
};

namespace detail {

void configure_timebase(
    registers& REG,
    emb::units::hz_f32 clk_freq,
    periodic_timer_config const& conf
);

} // namespace detail

template<some_general_purpose_timer Tim>
class periodic_timer {
public:
  using timer_instance = Tim;
private:
  static inline registers& REG = timer_instance::REG;

  static constexpr nvic::irq_number const update_irqn_ =
      timer_instance::update_irqn;

  emb::units::sec_f32 period_;
public:
  periodic_timer(periodic_timer const&) = delete;
  periodic_timer& operator=(periodic_timer const&) = delete;
  periodic_timer(periodic_timer&&) = delete;
  periodic_timer& operator=(periodic_timer&&) = delete;

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

    // Trigger output
    if constexpr (some_master_timer_instance<Tim>) {
      if (conf.trgo) {
        emb::mmio::write<TMR_CTRL2_MMSEL>(
            REG.CTRL2,
            std::to_underlying(*conf.trgo)
        );
      }
    }

    // Interrupt configuration
    emb::mmio::set<TMR_DIEN_UIEN>(REG.DIEN);
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
    emb::mmio::set<TMR_CTRL1_CNTEN>(REG.CTRL1);
  }

  void disable_counter() {
    emb::mmio::clear<TMR_CTRL1_CNTEN>(REG.CTRL1);
  }
};

} // namespace apm32::f4::tim
