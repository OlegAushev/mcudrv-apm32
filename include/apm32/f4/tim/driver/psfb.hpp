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

struct psfb_pwm_config {
  emb::units::hz_f32 frequency;
  emb::chrono::nanoseconds_i32 deadtime;
  clock_division clkdiv;
  std::optional<uint16_t> prescaler;
  nvic::irq_priority update_irq_priority;
  nvic::irq_priority break_irq_priority;
  trigger_output trgo;
};

struct psfb_config {
  psfb_pwm_config pwm;
  std::array<output_pin_config, 2> hi_pins;
  std::array<output_pin_config, 2> lo_pins;
  std::optional<break_pin_config> bk_pin;
};

namespace detail {

void configure_psfb_timebase(
    registers& regs,
    emb::units::hz_f32 clk_freq,
    psfb_pwm_config const& conf
);

// Force inactive level = 0b100, Toggle mode = 0b011
inline constexpr uint32_t oc_mode_force_inactive = 0b100u;
inline constexpr uint32_t oc_mode_toggle = 0b011u;

template<advanced_timer Tim, timer_channel_instance Ch>
  requires(emb::same_as_any<Ch, channel1, channel2>)
void configure_psfb_channel() {
  registers& regs = Tim::regs;

  switch (Ch::idx) {
  case channel_idx::ch1:
    // force inactive level before configuration
    emb::mmio::modify(regs.CCM1,
        emb::mmio::bits<TMR_CCM1_OC1PEN>(1u),
        emb::mmio::bits<TMR_CCM1_OC1MOD>(oc_mode_force_inactive)
    );
    emb::mmio::modify(regs.CCM1,
        emb::mmio::bits<TMR_CCM1_OC1MOD>(oc_mode_toggle)
    );
    emb::mmio::modify(regs.CCEN,
        emb::mmio::bits<TMR_CCEN_CC1EN>(1u),
        emb::mmio::bits<TMR_CCEN_CC1NEN>(1u),
        emb::mmio::bits<TMR_CCEN_CC1POL>(0u),
        emb::mmio::bits<TMR_CCEN_CC1NPOL>(0u)
    );
    emb::mmio::modify(regs.CTRL2,
        emb::mmio::bits<TMR_CTRL2_OC1OIS>(0u),
        emb::mmio::bits<TMR_CTRL2_OC1NOIS>(0u)
    );
    regs.CC1 = 0;
    break;
  case channel_idx::ch2:
    emb::mmio::modify(regs.CCM1,
        emb::mmio::bits<TMR_CCM1_OC2PEN>(1u),
        emb::mmio::bits<TMR_CCM1_OC2MOD>(oc_mode_force_inactive)
    );
    emb::mmio::modify(regs.CCM1,
        emb::mmio::bits<TMR_CCM1_OC2MOD>(oc_mode_toggle)
    );
    emb::mmio::modify(regs.CCEN,
        emb::mmio::bits<TMR_CCEN_CC2EN>(1u),
        emb::mmio::bits<TMR_CCEN_CC2NEN>(1u),
        emb::mmio::bits<TMR_CCEN_CC2POL>(0u),
        emb::mmio::bits<TMR_CCEN_CC2NPOL>(0u)
    );
    emb::mmio::modify(regs.CTRL2,
        emb::mmio::bits<TMR_CTRL2_OC2OIS>(0u),
        emb::mmio::bits<TMR_CTRL2_OC2NOIS>(0u)
    );
    regs.CC2 = 0;
    break;
  case channel_idx::ch3:
    std::unreachable();
    break;
  case channel_idx::ch4:
    std::unreachable();
    break;
  }
}

} // namespace detail

template<advanced_timer Tim>
class psfb : public emb::singleton<psfb<Tim>> {
public:
  using timer_instance = Tim;
  using counter_type = Tim::counter_type;
  static constexpr size_t LegCount = 2;
  using dutycycle_type = std::array<emb::unsigned_pu, LegCount>;
private:
  static inline registers& regs_ = timer_instance::regs;
  static inline nvic::irq_number const update_irqn_ =
      timer_instance::update_irqn;
  static inline nvic::irq_number const break_irqn_ = timer_instance::break_irqn;
  static inline std::array<uint32_t volatile*, 4> const compare_regs_ = {
      &regs_.CC1, &regs_.CC2, &regs_.CC3, &regs_.CC4
  };

  emb::units::hz_f32 timebase_freq_;
  emb::units::hz_f32 min_freq_;
  emb::units::hz_f32 max_freq_;

  emb::units::sec_f32 period_;
  emb::units::sec_f32 deadtime_;
  emb::unsigned_pu overlap_{0};

  std::array<std::optional<gpio::alternate_pin>, LegCount> hi_pins_;
  std::array<std::optional<gpio::alternate_pin>, LegCount> lo_pins_;
  std::optional<gpio::alternate_pin> bk_pin_;
public:
  psfb(psfb_config cfg) {
    period_ = 1.f / cfg.pwm.frequency;
    deadtime_ = emb::units::sec_f32{float(cfg.pwm.deadtime.count()) / 1E9f};

    if (!cfg.pwm.prescaler.has_value()) {
      cfg.pwm.prescaler = calculate_prescaler<timer_instance>(
          cfg.pwm.frequency,
          counter_mode::up
      );
    }

    timebase_freq_ =
        timer_instance::template clock_frequency<emb::units::hz_f32>()
        / static_cast<float>(cfg.pwm.prescaler.value() + 1);

    min_freq_ = timebase_freq_
              / (std::numeric_limits<counter_type>::max() + 1);
    max_freq_ = timebase_freq_;

    timer_instance::enable_clock();

    detail::configure_psfb_timebase(
        regs_,
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        cfg.pwm
    );

    if (cfg.bk_pin.has_value()) {
      bk_pin_.emplace(
          detail::make_break_input_gpio_config<timer_instance>(
              cfg.bk_pin.value()
          )
      );
    }
    detail::configure_bdt(
        regs_,
        timer_instance::template clock_frequency<emb::units::hz_f32>(),
        cfg.pwm.deadtime,
        cfg.pwm.clkdiv,
        cfg.bk_pin
    );

    emb::unroll<LegCount>([&]<size_t I>() {
      detail::configure_psfb_channel<
          timer_instance,
          tim::channel_at<I>>();
      hi_pins_[I].emplace(
          detail::make_output_gpio_config<timer_instance>(cfg.hi_pins[I])
      );
      lo_pins_[I].emplace(
          detail::make_output_gpio_config<timer_instance>(cfg.lo_pins[I])
      );
    });

    // Trigger output
    switch (cfg.pwm.trgo) {
    case trigger_output::none:
      break;
    case trigger_output::update:
      emb::mmio::write(regs_.CTRL2, TMR_CTRL2_MMSEL, 0b010u);
      break;
    }

    // Interrupt configuration
    emb::mmio::set(regs_.DIEN, TMR_DIEN_UIEN);
    set_irq_priority(update_irqn_, cfg.pwm.update_irq_priority);
    if (bk_pin_) {
      emb::mmio::set(regs_.DIEN, TMR_DIEN_BRKIEN);
      set_irq_priority(break_irqn_, cfg.pwm.break_irq_priority);
    }
  }

  registers& regs() {
    return regs_;
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
    regs_.AUTORLD = uint32_t(timebase_freq_ / (2 * freq)) - 1;
    set_overlap(overlap_);
    period_ = 1.f / freq;
  }

  bool active() const {
    return emb::mmio::test_any(regs_.BDT, TMR_BDT_MOEN);
  }

  bool bad() const {
    if (!bk_pin_) {
      return false;
    }
    return uint32_t(std::to_underlying(bk_pin_->read_level())) ==
           emb::mmio::read(regs_.BDT, TMR_BDT_BRKPOL);
  }

  void start() {
    if (bk_pin_) {
      acknowledge_break<timer_instance>();
      emb::mmio::set(regs_.DIEN, TMR_DIEN_BRKIEN);
    }
    emb::mmio::set(regs_.BDT, TMR_BDT_MOEN);
  }

  void stop() {
    emb::mmio::clear(regs_.BDT, TMR_BDT_MOEN);
    if (bk_pin_) {
      emb::mmio::clear(regs_.DIEN, TMR_DIEN_BRKIEN);
    }
  }

  dutycycle_type dutycycle() const {
    dutycycle_type dutycycle;
    float const reload_val = static_cast<float>(regs_.AUTORLD);
    emb::unroll<LegCount>([&]<size_t I>() {
      dutycycle[I] = emb::unsigned_pu{
          static_cast<float>(*compare_regs_[I]) / reload_val
      };
    });
    return dutycycle;
  }

  void set_dutycycle(dutycycle_type const& dutycycle) {
    float const reload_val = static_cast<float>(regs_.AUTORLD);
    emb::unroll<LegCount>([&]<size_t I>() {
      *compare_regs_[I] =
          static_cast<uint32_t>(dutycycle[I].value() * reload_val);
    });
  }

  void set_overlap(emb::unsigned_pu overlap) {
    auto arr_v = regs_.AUTORLD;
    auto mn = min_ccr_v();
    auto mx = max_ccr_v(arr_v);

    auto ccr2_v = std::clamp(
        mx - counter_type(overlap.value() * float(mx - mn)),
        mn,
        mx
    );

    regs_.CC1 = mx;
    regs_.CC2 = ccr2_v;
    overlap_ = overlap;
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

    overlap_ = emb::unsigned_pu{0};
    auto arr_v = regs_.AUTORLD;
    regs_.CC1 = max_ccr_v(arr_v);
    regs_.CC2 = max_ccr_v(arr_v);

    enable_counter<timer_instance>();
  }
private:
  static counter_type min_ccr_v() {
    return 1;
  }

  static counter_type max_ccr_v(counter_type arr_value) {
    return arr_value - 1;
  }
};

} // namespace pwm
} // namespace tim
} // namespace f4
} // namespace apm32
