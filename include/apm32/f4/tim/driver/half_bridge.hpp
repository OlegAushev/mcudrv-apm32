#pragma once

#include <apm32/f4/tim/driver/pwm.hpp>
#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>
#include <apm32/f4/tim/timer_utils.hpp>

#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/chrono.hpp>
#include <emb/math.hpp>
#include <emb/mmio.hpp>
#include <emb/singleton.hpp>
#include <emb/units.hpp>

#include <cassert>
#include <chrono>
#include <limits>
#include <optional>
#include <utility>

namespace apm32 {
namespace f4 {
namespace tim {
namespace pwm {

struct half_bridge_pwm_config {
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
  half_bridge_pwm_config pwm;
  std::array<output_pin_config, LegCount> hi_pins;
  std::array<output_pin_config, LegCount> lo_pins;
  std::optional<break_pin_config> bk_pin;
};

namespace detail {

void configure_half_bridge_timebase(
    registers& REG,
    emb::units::hz_f32 clk_freq,
    half_bridge_pwm_config const& conf
);

// PWM1 mode = 0b110
inline constexpr uint32_t oc_mode_pwm1 = 0b110u;

template<some_advanced_timer Tim, some_timer_channel_instance Ch>
  requires(!std::same_as<Ch, channel4>)
void configure_half_bridge_channel() {
  registers& REG = Tim::REG;

  switch (Ch::idx) {
  case channel_idx::ch1:
    emb::mmio::modify(REG.CCM1,
        emb::mmio::bits<TMR_CCM1_OC1PEN>(1u),
        emb::mmio::bits<TMR_CCM1_OC1MOD>(oc_mode_pwm1)
    );
    emb::mmio::modify(REG.CCEN,
        emb::mmio::bits<TMR_CCEN_CC1EN>(1u),
        emb::mmio::bits<TMR_CCEN_CC1NEN>(1u),
        emb::mmio::bits<TMR_CCEN_CC1POL>(0u),
        emb::mmio::bits<TMR_CCEN_CC1NPOL>(0u)
    );
    emb::mmio::modify(REG.CTRL2,
        emb::mmio::bits<TMR_CTRL2_OC1OIS>(0u),
        emb::mmio::bits<TMR_CTRL2_OC1NOIS>(0u)
    );
    REG.CC1 = 0;
    break;
  case channel_idx::ch2:
    emb::mmio::modify(REG.CCM1,
        emb::mmio::bits<TMR_CCM1_OC2PEN>(1u),
        emb::mmio::bits<TMR_CCM1_OC2MOD>(oc_mode_pwm1)
    );
    emb::mmio::modify(REG.CCEN,
        emb::mmio::bits<TMR_CCEN_CC2EN>(1u),
        emb::mmio::bits<TMR_CCEN_CC2NEN>(1u),
        emb::mmio::bits<TMR_CCEN_CC2POL>(0u),
        emb::mmio::bits<TMR_CCEN_CC2NPOL>(0u)
    );
    emb::mmio::modify(REG.CTRL2,
        emb::mmio::bits<TMR_CTRL2_OC2OIS>(0u),
        emb::mmio::bits<TMR_CTRL2_OC2NOIS>(0u)
    );
    REG.CC2 = 0;
    break;
  case channel_idx::ch3:
    emb::mmio::modify(REG.CCM2,
        emb::mmio::bits<TMR_CCM2_OC3PEN>(1u),
        emb::mmio::bits<TMR_CCM2_OC3MOD>(oc_mode_pwm1)
    );
    emb::mmio::modify(REG.CCEN,
        emb::mmio::bits<TMR_CCEN_CC3EN>(1u),
        emb::mmio::bits<TMR_CCEN_CC3NEN>(1u),
        emb::mmio::bits<TMR_CCEN_CC3POL>(0u),
        emb::mmio::bits<TMR_CCEN_CC3NPOL>(0u)
    );
    emb::mmio::modify(REG.CTRL2,
        emb::mmio::bits<TMR_CTRL2_OC3OIS>(0u),
        emb::mmio::bits<TMR_CTRL2_OC3NOIS>(0u)
    );
    REG.CC3 = 0;
    break;
  case channel_idx::ch4:
    std::unreachable();
    break;
  }
}

} // namespace detail

template<some_advanced_timer Tim, size_t LegCount = 1>
class half_bridge : public emb::singleton<half_bridge<Tim, LegCount>> {
public:
  using timer_instance = Tim;
  using counter_type = Tim::counter_type;
  using dutycycle_type = std::array<emb::unsigned_pu, LegCount>;
private:
  static inline registers& REG = timer_instance::REG;
  static inline nvic::irq_number const update_irqn_ =
      timer_instance::update_irqn;
  static inline nvic::irq_number const break_irqn_ = timer_instance::break_irqn;
  static inline std::array<uint32_t volatile*, 4> const CCR_REGS = {
      &REG.CC1, &REG.CC2, &REG.CC3, &REG.CC4
  };

  emb::units::hz_f32 timebase_freq_;
  emb::units::hz_f32 min_freq_;
  emb::units::hz_f32 max_freq_;

  emb::units::sec_f32 period_;
  emb::units::sec_f32 deadtime_;

  std::array<std::optional<gpio::alternate_pin>, LegCount> hi_pins_;
  std::array<std::optional<gpio::alternate_pin>, LegCount> lo_pins_;
  std::optional<gpio::alternate_pin> bk_pin_;
public:
  half_bridge(half_bridge_config<LegCount> conf) {
    period_ = 1.f / conf.pwm.frequency;
    deadtime_ = emb::units::sec_f32{float(conf.pwm.deadtime.count()) / 1E9f};

    // use prescaler from config or calculate it from required pwm frequency
    if (!conf.pwm.prescaler.has_value()) {
      conf.pwm.prescaler = calculate_prescaler<timer_instance>(
          conf.pwm.frequency,
          counter_mode::updown
      );
    }

    timebase_freq_ =
        timer_instance::template clock_frequency<emb::units::hz_f32>()
        / static_cast<float>(conf.pwm.prescaler.value() + 1);

    min_freq_ = timebase_freq_
              / (2.f * std::numeric_limits<counter_type>::max());
    max_freq_ = timebase_freq_ / 2.f;

    timer_instance::enable_clock();

    detail::configure_half_bridge_timebase(
        REG,
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        conf.pwm
    );

    if (conf.bk_pin.has_value()) {
      bk_pin_.emplace(
          detail::make_break_input_gpio_config<timer_instance>(
              conf.bk_pin.value()
          )
      );
    }
    detail::configure_bdt(
        REG,
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        conf.pwm.deadtime,
        conf.pwm.clkdiv,
        conf.bk_pin
    );

    emb::unroll<LegCount>([&]<size_t I>() {
      detail::configure_half_bridge_channel<
          timer_instance,
          tim::channel_at<I>>();
      hi_pins_[I].emplace(
          detail::make_output_gpio_config<timer_instance>(conf.hi_pins[I])
      );
      lo_pins_[I].emplace(
          detail::make_output_gpio_config<timer_instance>(conf.lo_pins[I])
      );
    });

    // Trigger output
    switch (conf.pwm.trgo) {
    case trigger_output::none:
      break;
    case trigger_output::update:
      emb::mmio::write(REG.CTRL2, TMR_CTRL2_MMSEL, 0b010u);
      break;
    }

    // Interrupt configuration
    emb::mmio::set(REG.DIEN, TMR_DIEN_UIEN);
    set_irq_priority(update_irqn_, conf.pwm.update_irq_priority);
    if (bk_pin_) {
      emb::mmio::set(REG.DIEN, TMR_DIEN_BRKIEN);
      set_irq_priority(break_irqn_, conf.pwm.break_irq_priority);
    }
  }

  emb::units::sec_f32 period() const {
    return period_;
  }

  emb::units::hz_f32 frequency() const {
    return 1.f / period_;
  }

  emb::units::sec_f32 deadtime() const {
    return deadtime_;
  }

  emb::units::hz_f32 min_frequency() const {
    return min_freq_;
  }

  emb::units::hz_f32 max_frequency() const {
    return max_freq_;
  }

  void set_frequency(emb::units::hz_f32 freq) {
    assert(freq >= min_freq_);
    assert(freq <= max_freq_);
    REG.AUTORLD = static_cast<uint32_t>((timebase_freq_ / freq) / 2);
    period_ = 1.f / freq;
  }

  bool active() const {
    return emb::mmio::test_any(REG.BDT, TMR_BDT_MOEN);
  }

  bool bad() const {
    if (!bk_pin_) {
      return false;
    }
    return uint32_t(std::to_underlying(bk_pin_->read_level())) ==
           emb::mmio::read(REG.BDT, TMR_BDT_BRKPOL);
  }

  void start() {
    if (bk_pin_) {
      acknowledge_break<timer_instance>();
      emb::mmio::set(REG.DIEN, TMR_DIEN_BRKIEN);
    }
    emb::mmio::set(REG.BDT, TMR_BDT_MOEN);
  }

  void stop() {
    emb::mmio::clear(REG.BDT, TMR_BDT_MOEN);
    if (bk_pin_) {
      // disable break interrupts to prevent instant call of BRK ISR
      emb::mmio::clear(REG.DIEN, TMR_DIEN_BRKIEN);
    }
  }

  count_direction timer_count_direction() const {
    return emb::mmio::test_any(REG.CTRL1, TMR_CTRL1_CNTDIR)
               ? count_direction::down
               : count_direction::up;
  }

  dutycycle_type dutycycle() const {
    dutycycle_type dutycycle;
    float const reload_val = static_cast<float>(REG.AUTORLD);
    emb::unroll<LegCount>([&]<size_t I>() {
      dutycycle[I] = emb::unsigned_pu{
          static_cast<float>(*CCR_REGS[I]) / reload_val
      };
    });
    return dutycycle;
  }

  void set_dutycycle(dutycycle_type const& dutycycle) {
    float const reload_val = static_cast<float>(REG.AUTORLD);
    emb::unroll<LegCount>([&]<size_t I>() {
      *CCR_REGS[I] =
          static_cast<uint32_t>(dutycycle[I].value() * reload_val);
    });
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
