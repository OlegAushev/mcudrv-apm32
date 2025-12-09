#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_spi.h>

#include <initializer_list>
#include <optional>
#include <vector>

namespace apm32 {
namespace f4 {
namespace spi {

using peripheral_registers = SPI_T;

inline constexpr size_t peripheral_count = 3;

inline std::array<peripheral_registers*, peripheral_count> const peripherals =
    {SPI1, SPI2, SPI3};

enum class peripheral_id : uint32_t { spi1, spi2, spi3 };

enum class bidimode_direction { rx, tx };

struct mosi_pin_config {
  gpio::port port;
  gpio::pin pin;
  GPIO_AF_T altfunc;
};

struct miso_pin_config {
  gpio::port port;
  gpio::pin pin;
  GPIO_AF_T altfunc;
};

struct clk_pin_config {
  gpio::port port;
  gpio::pin pin;
  GPIO_AF_T altfunc;
};

struct hardware_ss_pin_config {
  gpio::port port;
  gpio::pin pin;
  GPIO_AF_T altfunc;
};

struct software_ss_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct config {
  SPI_Config_T hal_config;
};

enum class interrupt_event { txe, rxne, err };

namespace detail {

class mosi_pin : public gpio::alternate_pin {
public:
  mosi_pin(mosi_pin_config const& conf);
};

class miso_pin : public gpio::alternate_pin {
public:
  miso_pin(miso_pin_config const& conf);
};

class clk_pin : public gpio::alternate_pin {
public:
  clk_pin(clk_pin_config const& conf);
};

class hardware_ss_pin : public gpio::alternate_pin {
public:
  hardware_ss_pin(hardware_ss_pin_config const& conf);
};

class software_ss_pin : public gpio::output_pin {
public:
  software_ss_pin(software_ss_pin_config const& conf);
};

inline std::array<void (*)(void), peripheral_count> const enable_clock = {
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SPI1); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI2); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI3); }
};

inline constexpr std::array<IRQn_Type, peripheral_count> irq_numbers =
    {SPI1_IRQn, SPI2_IRQn, SPI3_IRQn};

} // namespace detail

class peripheral
    : public core::peripheral<peripheral, peripheral_id, peripheral_count> {
public:
  using peripheral_type =
      core::peripheral<peripheral, peripheral_id, peripheral_count>;
private:
  peripheral_id const id_;
  peripheral_registers* const regs_;
  detail::mosi_pin mosi_pin_;
  detail::miso_pin miso_pin_;
  detail::clk_pin clk_pin_;
  std::unique_ptr<detail::hardware_ss_pin> ss_pin_;
  std::vector<std::unique_ptr<detail::software_ss_pin>> ss_pins_;
public:
  peripheral(
      peripheral_id id,
      mosi_pin_config const& mosi_pinconf,
      miso_pin_config const& miso_pinconf,
      clk_pin_config const& clk_pinconf,
      hardware_ss_pin_config const& ss_pinconf,
      config conf
  );

  peripheral(
      peripheral_id id,
      mosi_pin_config const& mosi_pinconf,
      miso_pin_config const& miso_pinconf,
      clk_pin_config const& clk_pinconf,
      std::initializer_list<software_ss_pin_config> ss_pinconfs,
      config conf
  );

  peripheral_id id() const {
    return id_;
  }

  peripheral_registers* regs() {
    return regs_;
  }

  void enable() {
    regs_->CTRL1_B.SPIEN = 1;
  }

  void disable() {
    regs_->CTRL1_B.SPIEN = 0;
  }

  bool busy() const {
    return regs_->STS_B.BSYFLG == 1;
  }

  bool rx_empty() const {
    return regs_->STS_B.RXBNEFLG == 0;
  }

  bool tx_empty() const {
    return regs_->STS_B.TXBEFLG == 1;
  }

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

  void set_bidirectional_mode(bidimode_direction dir) {
    switch (dir) {
    case bidimode_direction::rx:
      regs_->CTRL1_B.BMOEN = 0;
      break;
    case bidimode_direction::tx:
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
  void configure_interrupts(
      std::initializer_list<interrupt_event> events,
      nvic::irq_priority priority
  );

  void enable_interrupts() {
    nvic::enable_irq(detail::irq_numbers[std::to_underlying(id_)]);
  }

  void disable_interrupts() {
    nvic::disable_irq(detail::irq_numbers[std::to_underlying(id_)]);
  }
private:
  static inline std::array<bool, peripheral_count> is_clock_enabled_{};
  static void enable_clock(peripheral_id id);
};

} // namespace spi
} // namespace f4
} // namespace apm32
