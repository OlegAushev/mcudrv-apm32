#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/tim/timdef.hpp>

#include <emblib/noncopyable.hpp>
#include <emblib/singleton.hpp>

namespace mcu {
inline namespace apm32 {
namespace tim {
namespace gen1 {

using Regs = TMR_T;

constexpr size_t periph_num{2};

enum class Peripheral : size_t {
  tim2,
  tim5
};

inline std::array<TMR_T*, periph_num> const regs = {TMR2, TMR5};

inline Peripheral toperipheral_(Regs const* reg) {
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

inline constexpr std::array<IRQn_Type, periph_num> irq_nums = {TMR2_IRQn,
                                                               TMR5_IRQn};

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
        regs_(gen1::regs[std::to_underlying(peripheral)]),
        mode_(mode) {
    _enable_clk(peripheral);
  }

  Peripheral peripheral() const { return peripheral_; }

  Regs* regs() { return regs_; }

  OpMode mode() const { return mode_; }

  void enable() { regs_->CTRL1_B.CNTEN = 1; }

  void disable() { regs_->CTRL1_B.CNTEN = 0; }
private:
  static inline std::array<bool, periph_num> _clk_enabled{};
  static inline std::array<void (*)(void), periph_num> enable_clk_ = {
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR2); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_TMR5); }};

  static void _enable_clk(Peripheral peripheral) {
    auto timer_idx = std::to_underlying(peripheral);
    if (_clk_enabled[timer_idx]) {
      return;
    }

    enable_clk_[timer_idx]();
    _clk_enabled[timer_idx] = true;
  }
};

} // namespace internal

} // namespace gen1
} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
