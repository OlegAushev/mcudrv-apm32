#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_spi.h>

#include <mcudrv-apm32/f4/apm32f4.hpp>
#include <mcudrv-apm32/f4/gpio/gpio.hpp>
#include <mcudrv-apm32/f4/system/system.hpp>

#include <emblib/noncopyable.hpp>
#include <emblib/singleton.hpp>

#include <initializer_list>
#include <optional>
#include <vector>

namespace mcu {
namespace apm32 {
namespace spi {

using Regs = SPI_T;

constexpr size_t periph_num{3};

enum class Peripheral : size_t {
  spi1,
  spi2,
  spi3
};

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

struct HwCsPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct SwCsPinConfig {
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

namespace detail {

inline std::array<Regs*, periph_num> const regs = {SPI1, SPI2, SPI3};

inline Peripheral get_peripheral(Regs const* reg) {
  return static_cast<Peripheral>(
      std::distance(regs.begin(), std::find(regs.begin(), regs.end(), reg)));
}

inline std::array<void (*)(void), periph_num> clk_enable_funcs = {
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SPI1); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI2); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI3); },
};

inline constexpr std::array<IRQn_Type, periph_num> irqn = {
    SPI1_IRQn, SPI2_IRQn, SPI3_IRQn};

} // namespace detail

class Module : public emb::singleton_array<Module, periph_num>,
               private emb::noncopyable {
private:
  Peripheral const peripheral_;
  Regs* const regs_;
  std::unique_ptr<gpio::AlternatePin> mosi_pin_;
  std::unique_ptr<gpio::AlternatePin> miso_pin_;
  std::unique_ptr<gpio::AlternatePin> clk_pin_;
  std::unique_ptr<gpio::AlternatePin> cs_pin_;
  std::vector<std::unique_ptr<gpio::DigitalOutput>> cs_pins_;

  static inline std::array<bool, periph_num> clk_enabled_{};
public:
  Module(Peripheral peripheral,
         MosiPinConfig const& mosi_pin_conf,
         MisoPinConfig const& miso_pin_conf,
         ClkPinConfig const& clk_pin_conf,
         HwCsPinConfig const& cs_pin_conf,
         Config const& config);

  Module(Peripheral peripheral,
         MosiPinConfig const& mosi_pin_conf,
         MisoPinConfig const& miso_pin_conf,
         ClkPinConfig const& clk_pin_conf,
         std::initializer_list<SwCsPinConfig> cs_pin_confs,
         Config const& config);

  Peripheral peripheral() const { return peripheral_; }

  Regs* reg() { return regs_; }

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

  void set_cs(size_t cs_idx = 0) {
    if (cs_idx >= cs_pins_.size()) {
      return;
    }
    cs_pins_[cs_idx]->set();
  }

  void reset_cs(size_t cs_idx = 0) {
    if (cs_idx >= cs_pins_.size()) {
      return;
    }
    cs_pins_[cs_idx]->reset();
  }
public:
  void init_interrupts(std::initializer_list<InterruptEvent> events,
                       IrqPriority priority);

  void enable_interrupts() {
    enable_irq(detail::irqn[std::to_underlying(peripheral_)]);
  }

  void disable_interrupts() {
    disable_irq(detail::irqn[std::to_underlying(peripheral_)]);
  }
protected:
  static void enable_clk(Peripheral peripheral);
  void init_mosi_miso_clk(MosiPinConfig const& mosi_pin_conf,
                          MisoPinConfig const& miso_pin_conf,
                          ClkPinConfig const& clk_pin_conf);
};

} // namespace spi
} // namespace apm32
} // namespace mcu

#endif
#endif
