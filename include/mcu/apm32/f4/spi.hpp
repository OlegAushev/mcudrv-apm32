#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_spi.h>

#include <mcu/apm32/f4/gpio.hpp>
#include <mcu/apm32/f4/system.hpp>

#include <emb/noncopyable.hpp>
#include <emb/singleton.hpp>

#include <initializer_list>
#include <optional>
#include <vector>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace spi {

using Regs = SPI_T;

constexpr size_t periph_num{3};

enum class Peripheral : size_t {
  spi1,
  spi2,
  spi3
};

inline std::array<Regs*, periph_num> const regs = {SPI1, SPI2, SPI3};

inline Peripheral get_peripheral(Regs const* reg) {
  return static_cast<Peripheral>(
      std::distance(regs.begin(), std::find(regs.begin(), regs.end(), reg)));
}

enum class Direction {
  rx,
  tx
};

struct MosiPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct MisoPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct ClkPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct HwSsPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct SwSsPinConfig {
  gpio::Port port;
  gpio::Pin pin;
};

struct Config {
  SPI_Config_T hal_config;
};

enum class InterruptEvent {
  txe,
  rxne,
  err
};

namespace internal {

class MosiPin : public gpio::AlternatePin {
public:
  MosiPin(MosiPinConfig const& conf);
};

class MisoPin : public gpio::AlternatePin {
public:
  MisoPin(MisoPinConfig const& conf);
};

class ClkPin : public gpio::AlternatePin {
public:
  ClkPin(ClkPinConfig const& conf);
};

class HwSsPin : public gpio::AlternatePin {
public:
  HwSsPin(HwSsPinConfig const& conf);
};

class SwSsPin : public gpio::Output {
public:
  SwSsPin(SwSsPinConfig const& conf);
};

inline constexpr std::array<IRQn_Type, periph_num> irqn = {
    SPI1_IRQn, SPI2_IRQn, SPI3_IRQn};

} // namespace internal

class Module : public emb::singleton_array<Module, periph_num>,
               private emb::noncopyable {
private:
  Peripheral const peripheral_;
  Regs* const regs_;
  internal::MosiPin mosi_pin_;
  internal::MisoPin miso_pin_;
  internal::ClkPin clk_pin_;
  std::unique_ptr<internal::HwSsPin> ss_pin_;
  std::vector<std::unique_ptr<internal::SwSsPin>> ss_pins_;
public:
  Module(Peripheral peripheral,
         MosiPinConfig const& mosi_pinconf,
         MisoPinConfig const& miso_pinconf,
         ClkPinConfig const& clk_pinconf,
         HwSsPinConfig const& ss_pinconf,
         Config const& config);

  Module(Peripheral peripheral,
         MosiPinConfig const& mosi_pinconf,
         MisoPinConfig const& miso_pinconf,
         ClkPinConfig const& clk_pinconf,
         std::initializer_list<SwSsPinConfig> ss_pinconfs,
         Config const& config);

  Peripheral peripheral() const { return peripheral_; }

  Regs* regs() { return regs_; }

  static Module* instance(Peripheral peripheral) {
    return emb::singleton_array<Module, periph_num>::instance(
        std::to_underlying(peripheral));
  }

  void enable() { regs_->CTRL1_B.SPIEN = 1; }

  void disable() { regs_->CTRL1_B.SPIEN = 0; }

  bool busy() const { return regs_->STS_B.BSYFLG == 1; }

  bool rx_empty() const { return regs_->STS_B.RXBNEFLG == 0; }

  bool tx_empty() const { return regs_->STS_B.TXBEFLG == 1; }

  exec_status put_data(uint16_t data) {
    if (!tx_empty()) {
      return exec_status::busy;
    }
    regs_->DATA_B.DATA = data;
    return exec_status::ok;
  }

  std::optional<uint16_t> get_data() const {
    if (rx_empty()) {
      return {};
    }
    uint16_t data = regs_->DATA_B.DATA;
    return {data};
  }

  void set_bidirectional_mode(Direction dir) {
    switch (dir) {
    case Direction::rx:
      regs_->CTRL1_B.BMOEN = 0;
      break;
    case Direction::tx:
      regs_->CTRL1_B.BMOEN = 1;
      break;
    }
  }

  void set_ss(size_t ss_idx = 0) {
    if (ss_idx >= ss_pins_.size()) {
      return;
    }
    ss_pins_[ss_idx]->set();
  }

  void reset_ss(size_t ss_idx = 0) {
    if (ss_idx >= ss_pins_.size()) {
      return;
    }
    ss_pins_[ss_idx]->reset();
  }
public:
  void init_interrupts(std::initializer_list<InterruptEvent> events,
                       IrqPriority priority);

  void enable_interrupts() {
    enable_irq(internal::irqn[std::to_underlying(peripheral_)]);
  }

  void disable_interrupts() {
    disable_irq(internal::irqn[std::to_underlying(peripheral_)]);
  }
private:
  static inline std::array<bool, periph_num> clk_enabled_{};
  static void enable_clk(Peripheral peripheral);
  static inline std::array<void (*)(void), periph_num> enable_clk_ = {
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SPI1); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI2); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI3); }};
};

} // namespace spi
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
#endif
