#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>
#include <apm32/f4/tim/timer_utils.hpp>

#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_tmr.h>

#include <emb/chrono.hpp>
#include <emb/math.hpp>
#include <emb/mmio.hpp>
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

void configure_bdt(
    emb::units::hz_f32 clk_freq,
    registers& regs,
    base_config const& conf,
    std::optional<break_pin_config> const& bk_pin
);

template<advanced_timer Tim, timer_channel_instance Ch>
  requires(!std::same_as<Ch, channel4>)
void configure_channel() {
  registers& regs = Tim::regs;

  TMR_OCConfig_T ch_config{};
  ch_config.mode = TMR_OC_MODE_PWM1;
  ch_config.outputState = TMR_OC_STATE_ENABLE;
  ch_config.outputNState = TMR_OC_NSTATE_ENABLE;
  ch_config.polarity = TMR_OC_POLARITY_HIGH;
  ch_config.nPolarity = TMR_OC_NPOLARITY_HIGH;
  ch_config.idleState = TMR_OC_IDLE_STATE_RESET;
  ch_config.nIdleState = TMR_OC_NIDLE_STATE_RESET;
  ch_config.pulse = 0;

  switch (Ch::idx) {
  case channel_idx::ch1:
    regs.CCM1_COMPARE_B.OC1PEN = 1;
    TMR_ConfigOC1(&regs, &ch_config);
    break;
  case channel_idx::ch2:
    regs.CCM1_COMPARE_B.OC2PEN = 1;
    TMR_ConfigOC2(&regs, &ch_config);
    break;
  case channel_idx::ch3:
    regs.CCM2_COMPARE_B.OC3PEN = 1;
    TMR_ConfigOC3(&regs, &ch_config);
    break;
  case channel_idx::ch4:
    std::unreachable();
    break;
  }
}

template<advanced_timer Tim>
[[nodiscard]] gpio::alternate_pin_config
make_break_input_config(break_pin_config const& bk_pin) {
  return gpio::alternate_pin_config{
      .port = bk_pin.port,
      .pin = bk_pin.pin,
      .pull = bk_pin.pull,
      .output_type = gpio::output_type::pushpull,
      .speed = gpio::speed::low,
      .altfunc = Tim::gpio_altfunc
  };
}

template<advanced_timer Tim>
[[nodiscard]] gpio::alternate_pin_config
make_output_config(output_pin_config const& pin) {
  return gpio::alternate_pin_config{
      .port = pin.port,
      .pin = pin.pin,
      .pull = gpio::pull::none,
      .output_type = gpio::output_type::pushpull,
      .speed = gpio::speed::medium,
      .altfunc = Tim::gpio_altfunc
  };
}

} // namespace detail

template<advanced_timer Tim, size_t LegCount = 1>
class half_bridge : public emb::singleton<half_bridge<Tim, LegCount>> {
public:
  using timer_instance = Tim;
  using reg_addr = timer_instance::reg_addr;
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
          detail::make_break_input_config<timer_instance>(conf.bk_pin.value())
      );
    }
    detail::configure_bdt(
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        regs_,
        conf.pwm,
        conf.bk_pin
    );

    [&]<size_t... I>(std::index_sequence<I...>) {
      ((detail::configure_channel<timer_instance, tim::channel_at<I>>(),
        hi_pins_[I].emplace(
            detail::make_output_config<timer_instance>(conf.hi_pins[I])
        ),
        lo_pins_[I].emplace(
            detail::make_output_config<timer_instance>(conf.lo_pins[I])
        )),
       ...);
    }(std::make_index_sequence<LegCount>{});

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
      acknowledge_break<timer_instance>();
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
    [&]<size_t... I>(std::index_sequence<I...>) {
      ((dutycycle[I] =
            emb::unsigned_pu{
                static_cast<float>(emb::mmio::reg<reg_addr::ccrx[I]>::read()) /
                reload_val
            }),
       ...);
    }(std::make_index_sequence<LegCount>{});
    return dutycycle;
  }

  void set_dutycycle(dutycycle_type const& dutycycle) {
    float const reload_val = static_cast<float>(regs_.AUTORLD);
    [&]<size_t... I>(std::index_sequence<I...>) {
      ((emb::mmio::reg<reg_addr::ccrx[I]>::write(
           static_cast<uint32_t>(dutycycle[I].value() * reload_val)
       )),
       ...);
    }(std::make_index_sequence<LegCount>{});
  }
public:
  void enable() {
    acknowledge_update<timer_instance>();
    nvic::clear_pending_irq(update_irqn_);
    nvic::enable_irq(update_irqn_);
    if (bk_pin_) {
      acknowledge_break<timer_instance>();
      nvic::clear_pending_irq(break_irqn_);
      nvic::enable_irq(break_irqn_);
    }
    enable_counter<timer_instance>();
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
