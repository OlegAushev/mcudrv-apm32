#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <emb/math.hpp>
#include "base.hpp"

#include <memory>
#include <utility>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace tim {
namespace adv {

struct PwmConfig {
  float freq;
  float deadtime_ns;
  bool arr_preload;
  TMR_BaseConfig_T hal_base_config;
  TMR_BDTConfig_T hal_bdt_config;
};

struct PwmChannelConfig {
  TMR_OCConfig_T hal_oc_config;
  TMR_OC_PRELOAD_T oc_preload;
};

class PwmTimer : public internal::AbstractTimer {
private:
  float freq_{0};
  float t_dts_ns_{0};
  float deadtime_{0};
  std::array<std::pair<std::unique_ptr<mcu::apm32::tim::internal::ChPin>,
                       std::unique_ptr<mcu::apm32::tim::internal::ChPin>>,
             4>
      ch_pins_{};
  std::unique_ptr<mcu::apm32::tim::internal::BkinPin> bkin_pin_{};
  bool brk_enabled_{false};
public:
  PwmTimer(Peripheral peripheral,
           PwmConfig const& conf,
           BkinPinConfig const* bkin_conf);

  static PwmTimer* instance(Peripheral peripheral) {
    assert(internal::AbstractTimer::instance(std::to_underlying(peripheral))
               ->mode() == OpMode::pwm_generation);
    return static_cast<PwmTimer*>(
        internal::AbstractTimer::instance(std::to_underlying(peripheral)));
  }

  void init_channel(Channel channel,
                    ChPinConfig const* ch_conf,
                    ChPinConfig const* chn_conf,
                    PwmChannelConfig const& conf);

  bool active() const { return regs_->BDT_B.MOEN == 1; }

  bool bad() const {
    if (!bkin_pin_) {
      return false;
    }
    return bkin_pin_->read() == emb::gpio::state::active;
  }

  void start() {
    if (brk_enabled_) {
      regs_->STS_B.BRKIFLG = 0;
      regs_->DIEN_B.BRKIEN = 1;
    }
    regs_->BDT_B.MOEN = 1;
  }

  void stop() {
    regs_->BDT_B.MOEN = 0;
    if (brk_enabled_) {
      // disable break interrupts to prevent instant call of BRK ISR
      regs_->DIEN_B.BRKIEN = 0;
    }
  }

  emb::unsigned_pu duty_cycle(Channel channel) const {
    switch (channel) {
    case Channel::channel1:
      return emb::unsigned_pu{float(regs_->CC1) / float(regs_->AUTORLD)};
    case Channel::channel2:
      return emb::unsigned_pu{float(regs_->CC2) / float(regs_->AUTORLD)};
    case Channel::channel3:
      return emb::unsigned_pu{float(regs_->CC3) / float(regs_->AUTORLD)};
    case Channel::channel4:
      return emb::unsigned_pu{float(regs_->CC4) / float(regs_->AUTORLD)};
    }
    return {};
  }

  void set_duty_cycle(Channel channel, emb::unsigned_pu duty_cycle) {
    uint32_t compare_value =
        uint32_t(duty_cycle.numval() * float(regs_->AUTORLD));
    switch (channel) {
    case Channel::channel1:
      write_reg(regs_->CC1, compare_value);
      break;
    case Channel::channel2:
      write_reg(regs_->CC2, compare_value);
      break;
    case Channel::channel3:
      write_reg(regs_->CC3, compare_value);
      break;
    case Channel::channel4:
      write_reg(regs_->CC4, compare_value);
      break;
    }
  }

  void set_duty_cycle(std::array<emb::unsigned_pu, 3> const& duty_cycle) {
    auto to_cmpval = [this](emb::unsigned_pu pu) -> uint32_t {
      return static_cast<uint32_t>(pu.numval() *
                                   static_cast<float>(this->regs_->AUTORLD));
    };

    write_reg(regs_->CC1, to_cmpval(duty_cycle[0]));
    write_reg(regs_->CC2, to_cmpval(duty_cycle[1]));
    write_reg(regs_->CC3, to_cmpval(duty_cycle[2]));
  }

  float freq() const { return freq_; }

  void init_update_interrupts(IrqPriority priority);

  void enable_update_interrupts() {
    regs_->STS_B.UIFLG = 0;
    clear_pending_irq(internal::up_irq_nums[std::to_underlying(peripheral_)]);
    enable_irq(internal::up_irq_nums[std::to_underlying(peripheral_)]);
  }

  void disable_update_interrupts() {
    disable_irq(internal::up_irq_nums[std::to_underlying(peripheral_)]);
  }

  void ack_update_interrupt() { regs_->STS_B.UIFLG = 0; }

  void init_break_interrupts(IrqPriority priority);

  void enable_break_interrupts() {
    regs_->STS_B.BRKIFLG = 0;
    clear_pending_irq(internal::brk_irq_nums[std::to_underlying(peripheral_)]);
    enable_irq(internal::brk_irq_nums[std::to_underlying(peripheral_)]);
  }

  void disable_break_interrupts() {
    disable_irq(internal::brk_irq_nums[std::to_underlying(peripheral_)]);
  }

  void ack_break_interrupt() { regs_->STS_B.BRKIFLG = 0; }
private:
  void init_bdt(PwmConfig const& config, BkinPinConfig const* pin_bkin);
};

} // namespace adv
} // namespace tim
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
#endif
