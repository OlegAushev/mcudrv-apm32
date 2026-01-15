#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>
#include <apm32/f4/tim/timer_utils.hpp>

#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_tmr.h>

#include <emb/chrono.hpp>
#include <emb/math.hpp>
#include <emb/singleton.hpp>
#include <emb/units.hpp>

#include <chrono>
#include <optional>
#include <utility>

namespace apm32 {
namespace f4 {
namespace tim {
namespace pwm {

struct output_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct break_pin_config {
  gpio::port port;
  gpio::pin pin;
  gpio::pull pull;
  emb::gpio::level active_level;
};

enum class trigger_output {
  none,
  update
};

struct base_config {
  emb::units::hz_f32 frequency;
  emb::chrono::nanoseconds_i32 deadtime;
  clock_division clkdiv;
  std::optional<uint16_t> prescaler;
  nvic::irq_priority update_irq_priority;
  nvic::irq_priority break_irq_priority;
  trigger_output trgo;
};

template<size_t LegCount = 1>
struct half_bridge_config {
  base_config pwm;
  std::array<output_pin_config, LegCount> hi_pins;
  std::array<output_pin_config, LegCount> lo_pins;
  std::optional<break_pin_config> bk_pin;
};

namespace detail {

void configure_timebase(
    emb::units::hz_f32 clk_freq,
    registers& regs,
    base_config const& conf
);

[[nodiscard]] gpio::alternate_pin_config
get_break_input_config(registers& regs, break_pin_config const& bk_pin);

void configure_bdt(
    emb::units::hz_f32 clk_freq,
    registers& regs,
    base_config const& conf,
    std::optional<break_pin_config> const& bk_pin
);

void configure_channel(registers& regs, channel ch);

[[nodiscard]] gpio::alternate_pin_config
get_output_config(registers& regs, output_pin_config const& pin);

} // namespace detail

template<advanced_timer Tim, size_t LegCount = 1>
class half_bridge : public emb::singleton<half_bridge<Tim, LegCount>> {
public:
  using timer_instance = Tim;
  using dutycycle_type = std::array<emb::unsigned_pu, LegCount>;
private:
  static inline registers& regs_ = timer_instance::regs;
  static inline nvic::irq_number const update_irqn_ =
      timer_instance::update_irqn;
  static inline nvic::irq_number const break_irqn_ = timer_instance::break_irqn;
  static inline std::array<uint32_t volatile*, 4> const compare_regs_ =
      timer_instance::ccr_regs;

  emb::units::hz_f32 freq_;
  emb::chrono::nanoseconds_i32 deadtime_;

  std::array<std::optional<gpio::alternate_pin>, LegCount> hi_pins_;
  std::array<std::optional<gpio::alternate_pin>, LegCount> lo_pins_;
  std::optional<gpio::alternate_pin> bk_pin_;
public:
  half_bridge(half_bridge_config<LegCount> conf) {
    freq_ = conf.pwm.frequency;
    deadtime_ = conf.pwm.deadtime;

    // use prescaler from config or calculate it from required pwm frequency
    if (!conf.pwm.prescaler.has_value()) {
      conf.pwm.prescaler = calculate_prescaler<timer_instance>(
          freq_,
          counter_mode::updown
      );
    }

    timer_instance::enable_clock();

    detail::configure_timebase(
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        regs_,
        conf.pwm
    );

    if (conf.bk_pin.has_value()) {
      bk_pin_.emplace(
          detail::get_break_input_config(regs_, conf.bk_pin.value())
      );
    }
    detail::configure_bdt(
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        regs_,
        conf.pwm,
        conf.bk_pin
    );

    for (auto i = 0uz; i < LegCount; ++i) {
      detail::configure_channel(regs_, static_cast<channel>(i));
      hi_pins_[i].emplace(detail::get_output_config(regs_, conf.hi_pins[i]));
      lo_pins_[i].emplace(detail::get_output_config(regs_, conf.lo_pins[i]));
    }

    // Trigger output
    switch (conf.pwm.trgo) {
    case trigger_output::none:
      break;
    case trigger_output::update:
      regs_.CTRL2_B.MMSEL = 0b010u;
      break;
    }

    // Interrupt configuration
    regs_.DIEN_B.UIEN = 1;
    set_irq_priority(update_irqn_, conf.pwm.update_irq_priority);
    if (bk_pin_) {
      regs_.DIEN_B.BRKIEN = 1;
      set_irq_priority(break_irqn_, conf.pwm.break_irq_priority);
    }
  }

  registers& regs() {
    return regs_;
  }

  emb::units::hz_f32 frequency() const {
    return freq_;
  }

  emb::chrono::nanoseconds_i32 deadtime() const {
    return deadtime_;
  }

  bool active() const {
    return regs_.BDT_B.MOEN == 1;
  }

  bool bad() const {
    if (!bk_pin_) {
      return false;
    }
    return std::to_underlying(bk_pin_->read_level()) == regs_.BDT_B.BRKPOL;
  }

  void start() {
    if (bk_pin_) {
      ack_break_interrupt();
      regs_.DIEN_B.BRKIEN = 1;
    }
    regs_.BDT_B.MOEN = 1;
  }

  void stop() {
    regs_.BDT_B.MOEN = 0;
    if (bk_pin_) {
      // disable break interrupts to prevent instant call of BRK ISR
      regs_.DIEN_B.BRKIEN = 0;
    }
  }

  count_direction timer_count_direction() const {
    return regs_.CTRL1_B.CNTDIR == 0 ? count_direction::up :
                                       count_direction::down;
  }

  dutycycle_type dutycycle() const {
    dutycycle_type dutycycle;
    float const reload_val = static_cast<float>(regs_.AUTORLD);
    for (auto leg = 0uz; auto& dc : dutycycle) {
      dc = static_cast<float>(*compare_regs_[leg++]) / reload_val;
    }
    return dutycycle;
  }

  void set_dutycycle(dutycycle_type const& dutycycle) {
    float const reload_val = static_cast<float>(regs_.AUTORLD);
    for (auto leg = 0uz; auto const& dc : dutycycle) {
      *compare_regs_[leg++] = static_cast<uint32_t>(dc.value() * reload_val);
    }
  }
public:
  void enable() {
    ack_update_interrupt();
    nvic::clear_pending_irq(update_irqn_);
    nvic::enable_irq(update_irqn_);
    if (bk_pin_) {
      ack_break_interrupt();
      nvic::clear_pending_irq(break_irqn_);
      nvic::enable_irq(break_irqn_);
    }
    enable_counter();
  }

  void ack_update_interrupt() {
    regs_.STS_B.UIFLG = 0;
  }

  void ack_break_interrupt() {
    regs_.STS_B.BRKIFLG = 0;
  }
private:
  void enable_counter() {
    regs_.CTRL1_B.CNTEN = 1;
  }

  void disable_counter() {
    regs_.CTRL1_B.CNTEN = 0;
  }
};

using full_bridge_config = half_bridge_config<2>;

template<typename Tim>
using full_bridge = half_bridge<Tim, 2>;

using three_phase_bridge_config = half_bridge_config<3>;

template<typename Tim>
using three_phase_bridge = half_bridge<Tim, 3>;

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
