#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/mmio.hpp>

#include <initializer_list>
#include <optional>
#include <vector>

namespace apm32 {
namespace f4 {
namespace spi {

using peripheral_registers = SPI_TypeDef;

inline constexpr size_t peripheral_count = 3;

inline std::array<peripheral_registers*, peripheral_count> const peripherals =
    {SPI1, SPI2, SPI3};

enum class peripheral_id : uint32_t { spi1, spi2, spi3 };

enum class bidimode_direction { rx, tx };

struct mosi_pin_config {
  gpio::port port;
  gpio::pin pin;
  uint32_t altfunc;
};

struct miso_pin_config {
  gpio::port port;
  gpio::pin pin;
  uint32_t altfunc;
};

struct clk_pin_config {
  gpio::port port;
  gpio::pin pin;
  uint32_t altfunc;
};

struct hardware_ss_pin_config {
  gpio::port port;
  gpio::pin pin;
  uint32_t altfunc;
};

struct software_ss_pin_config {
  gpio::port port;
  gpio::pin pin;
};

enum class mode : uint32_t { slave = 0, master = 1 };
enum class clock_polarity : uint32_t { low = 0, high = 1 };
enum class clock_phase : uint32_t { first_edge = 0, second_edge = 1 };
enum class data_length : uint32_t { bits_8 = 0, bits_16 = 1 };
enum class bit_order : uint32_t { msb_first = 0, lsb_first = 1 };
enum class nss_mode : uint32_t { hardware = 0, software = 1 };
enum class direction : uint32_t {
  full_duplex = 0b00,
  rx_only     = 0b01,
  bidi_rx     = 0b10,
  bidi_tx     = 0b11,
};
enum class baudrate_divisor : uint32_t {
  div2 = 0b000, div4 = 0b001, div8 = 0b010, div16 = 0b011,
  div32 = 0b100, div64 = 0b101, div128 = 0b110, div256 = 0b111,
};

struct config {
  spi::mode mode;
  spi::clock_polarity cpol;
  spi::clock_phase cpha;
  spi::data_length data_length;
  spi::bit_order bit_order;
  spi::nss_mode nss;
  spi::direction direction;
  spi::baudrate_divisor baudrate;

  constexpr uint32_t ctrl1() const {
    auto dir = std::to_underlying(direction);
    return (std::to_underlying(cpha) << SPI_CTRL1_CPHA_Pos)
         | (std::to_underlying(cpol) << SPI_CTRL1_CPOL_Pos)
         | (std::to_underlying(mode) << SPI_CTRL1_MSMCFG_Pos)
         | (std::to_underlying(baudrate) << SPI_CTRL1_BRSEL_Pos)
         | (std::to_underlying(bit_order) << SPI_CTRL1_LSBSEL_Pos)
         | (nss == nss_mode::software ? (SPI_CTRL1_SSEN | SPI_CTRL1_ISSEL) : 0u)
         | ((dir & 0b01u) << SPI_CTRL1_RXOMEN_Pos)
         | ((dir >> 1) << SPI_CTRL1_BMEN_Pos)
         | (std::to_underlying(data_length) << SPI_CTRL1_DFLSEL_Pos);
  }
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

inline constexpr std::array<uint32_t, peripheral_count> clock_bits = {
    RCM_APB2CLKEN_SPI1EN,
    RCM_APB1CLKEN_SPI2EN,
    RCM_APB1CLKEN_SPI3EN,
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
    emb::mmio::set(regs_->CTRL1, SPI_CTRL1_SPIEN);
  }

  void disable() {
    emb::mmio::clear(regs_->CTRL1, SPI_CTRL1_SPIEN);
  }

  bool busy() const {
    return emb::mmio::test_any(regs_->STS, SPI_STS_BSYFLG);
  }

  bool rx_empty() const {
    return !emb::mmio::test_any(regs_->STS, SPI_STS_RXBNEFLG);
  }

  bool tx_empty() const {
    return emb::mmio::test_any(regs_->STS, SPI_STS_TXBEFLG);
  }

  exec_status put_data(uint16_t data) {
    if (!tx_empty()) {
      return exec_status::busy;
    }
    regs_->DATA = data;
    return exec_status::ok;
  }

  std::optional<uint16_t> get_data() const {
    if (rx_empty()) {
      return {};
    }
    uint16_t data = static_cast<uint16_t>(regs_->DATA);
    return {data};
  }

  void set_bidirectional_mode(bidimode_direction dir) {
    switch (dir) {
    case bidimode_direction::rx:
      emb::mmio::clear(regs_->CTRL1, SPI_CTRL1_BMOEN);
      break;
    case bidimode_direction::tx:
      emb::mmio::set(regs_->CTRL1, SPI_CTRL1_BMOEN);
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
