#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/tim/timdef.hpp>

#include <emblib/noncopyable.hpp>
#include <emblib/singleton.hpp>

namespace mcu {
namespace apm32 {
namespace tim {
namespace adv {

using Regs = TMR_T;

constexpr size_t periph_num{2};

enum class Peripheral : size_t {
  tim1,
  tim8
};

inline std::array<Regs*, periph_num> const regs = {TMR1, TMR8};

inline Peripheral get_peripheral(Regs const* reg) {
  return static_cast<Peripheral>(
      std::distance(regs.begin(), std::find(regs.begin(), regs.end(), reg)));
}

constexpr size_t channel_num{4};

enum class Channel : size_t {
  channel1,
  channel2,
  channel3,
  channel4,
};

namespace internal {

inline constexpr std::array<IRQn_Type, periph_num> up_irq_nums = {
    TMR1_UP_TMR10_IRQn, TMR8_UP_TMR13_IRQn};
inline constexpr std::array<IRQn_Type, periph_num> brk_irq_nums = {
    TMR1_BRK_TMR9_IRQn, TMR8_BRK_TMR12_IRQn};

class AbstractTimer : public emb::singleton_array<AbstractTimer, periph_num>,
                      private emb::noncopyable {
protected:
  Peripheral const peripheral_;
  Regs* const regs_;
  OpMode const mode_;
public:
  AbstractTimer(Peripheral peripheral, OpMode mode)
      : emb::singleton_array<AbstractTimer, periph_num>(
            this, std::to_underlying(peripheral)),
        peripheral_(peripheral),
        regs_(adv::regs[std::to_underlying(peripheral)]),
        mode_(mode) {
    _enable_clk(peripheral);
  }

  Peripheral peripheral() const { return peripheral_; }

  Regs* regs() { return regs_; }

  OpMode mode() const { return mode_; }

  CountDir dir() const {
    if (regs_->CTRL1_B.CNTDIR == 1) {
      return CountDir::down;
    }
    return CountDir::up;
  }

  void enable() { regs_->CTRL1_B.CNTEN = 1; }

  void disable() { regs_->CTRL1_B.CNTEN = 0; }
private:
  static inline std::array<bool, periph_num> clk_enabled_{};
  static inline std::array<void (*)(void), periph_num> enable_clk_ = {
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR1); },
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR8); }};

  static void _enable_clk(Peripheral peripheral) {
    auto timer_idx = std::to_underlying(peripheral);
    if (clk_enabled_[timer_idx]) {
      return;
    }

    enable_clk_[timer_idx]();
    clk_enabled_[timer_idx] = true;
  }
};

} // namespace internal

} // namespace adv
} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
