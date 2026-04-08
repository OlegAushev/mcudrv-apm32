#pragma once

#include <apm32/f4/spi/spi_instances.hpp>
#include <apm32/f4/spi/spi_types.hpp>
#include <apm32/f4/spi/spi_utils.hpp>

#include <emb/chrono.hpp>

#include <array>
#include <chrono>
#include <expected>
#include <optional>

namespace apm32 {
namespace f4 {
namespace spi {

template<size_t SlaveCount>
struct blocking_master_config {
  mosi_pin_config mosi_pin;
  miso_pin_config miso_pin;
  clk_pin_config clk_pin;
  std::array<ss_pin_config, SlaveCount> ss_pins;
  emb::units::hz_f32 clk_frequency;
  clock_polarity cpol;
  clock_phase cpha;
  spi::bit_order bit_order;
};

template<
    some_spi_instance Instance,
    frame_format FrameFormat,
    size_t SlaveCount>
class blocking_master {
public:
  using spi_instance = Instance;
private:
  static inline registers& REG = spi_instance::REG;

  std::optional<gpio::alternate_pin> mosi_pin_;
  std::optional<gpio::alternate_pin> miso_pin_;
  std::optional<gpio::alternate_pin> clk_pin_;
  std::array<std::optional<gpio::output_pin>, SlaveCount> ss_pins_;
public:
  blocking_master(blocking_master_config<SlaveCount> const& config) {
    spi_instance::enable_clock();

    emb::mmio::modify(
        REG.CTRL1,
        emb::mmio::bits<SPI_CTRL1_CPHA>(std::to_underlying(config.cpha)),
        emb::mmio::bits<SPI_CTRL1_CPOL>(std::to_underlying(config.cpol)),
        emb::mmio::bits<SPI_CTRL1_MSMCFG>(1),
        emb::mmio::bits<SPI_CTRL1_BRSEL>(std::to_underlying(
            calculate_prescaler<spi_instance>(config.clk_frequency)
        )),
        emb::mmio::bits<SPI_CTRL1_LSBSEL>(std::to_underlying(config.bit_order)),
        emb::mmio::bits<SPI_CTRL1_SSEN>(1u),
        emb::mmio::bits<SPI_CTRL1_ISSEL>(1u),
        emb::mmio::bits<SPI_CTRL1_RXOMEN>(0u),
        emb::mmio::bits<SPI_CTRL1_BMEN>(0u),
        emb::mmio::bits<SPI_CTRL1_DFLSEL>(
            std::is_same_v<FrameFormat, uint8_t> ? 0u : 1u
        )
    );

    mosi_pin_.emplace(
        gpio::alternate_pin_config{
            .port = config.mosi_pin.port,
            .pin = config.mosi_pin.pin,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = pin_speed(config.clk_frequency),
            .altfunc = spi_instance::gpio_altfunc
        }
    );

    miso_pin_.emplace(
        gpio::alternate_pin_config{
            .port = config.miso_pin.port,
            .pin = config.miso_pin.pin,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = pin_speed(config.clk_frequency),
            .altfunc = spi_instance::gpio_altfunc
        }
    );

    clk_pin_.emplace(
        gpio::alternate_pin_config{
            .port = config.clk_pin.port,
            .pin = config.clk_pin.pin,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = pin_speed(config.clk_frequency),
            .altfunc = spi_instance::gpio_altfunc
        }
    );

    for (auto i = 0uz; i < ss_pins_.size(); ++i) {
      ss_pins_[i].emplace(
          gpio::output_pin_config{
              .port = config.ss_pins[i].port,
              .pin = config.ss_pins[i].pin,
              .pull = gpio::pull::none,
              .output_type = gpio::output_type::pushpull,
              .speed = pin_speed(config.clk_frequency),
              .active_level = emb::gpio::level::low
          }
      );
      ss_pins_[i]->reset();
    }

    enable<spi_instance>();
  }

  void select()
    requires(SlaveCount == 1) {
    ss_pins_[0]->set();
  }

  template<size_t Idx>
  void select() {
    std::get<Idx>(ss_pins_)->set();
  }

  void select(size_t idx) {
    ss_pins_[idx]->set();
  }

  void release()
    requires(SlaveCount == 1) {
    ss_pins_[0]->reset();
  }

  template<size_t Idx>
  void release() {
    std::get<Idx>(ss_pins_)->reset();
  }

  void release(size_t idx) {
    ss_pins_[idx]->reset();
  }

  bool busy() const {
    return emb::mmio::test_any(REG.STS, SPI_STS_BSYFLG);
  }

  bool rx_empty() const {
    return !emb::mmio::test_any(REG.STS, SPI_STS_RXBNEFLG);
  }

  bool tx_empty() const {
    return emb::mmio::test_any(REG.STS, SPI_STS_TXBEFLG);
  }

  bool can_get() const {
    return !rx_empty();
  }

  bool can_put() const {
    return tx_empty();
  }

  auto try_get() const -> std::optional<FrameFormat> {
    if (can_get()) {
      return static_cast<FrameFormat>(REG.DATA);
    }
    return {};
  }

  auto try_put(FrameFormat data) -> std::optional<FrameFormat> {
    if (can_put()) {
      REG.DATA = data;
      return data;
    }
    return {};
  }

  auto get(std::chrono::milliseconds timeout) const
      -> std::expected<FrameFormat, error> {
    if (emb::mmio::test_any(REG.STS, SPI_STS_OVRFLG)) {
      clear_overrun();
      return std::unexpected(error::overrun);
    }
    emb::chrono::timeout t(timeout);
    while (rx_empty()) {
      if (t.expired()) return std::unexpected(error::timeout);
    }
    return static_cast<FrameFormat>(REG.DATA);
  }

  auto put(FrameFormat data, std::chrono::milliseconds timeout)
      -> std::expected<void, error> {
    if (emb::mmio::test_any(REG.STS, SPI_STS_OVRFLG)) {
      clear_overrun();
      return std::unexpected(error::overrun);
    }
    emb::chrono::timeout t(timeout);
    while (!tx_empty()) {
      if (t.expired()) return std::unexpected(error::timeout);
    }
    REG.DATA = data;
    return {};
  }

  auto transfer(FrameFormat tx_data, std::chrono::milliseconds timeout)
      -> std::expected<FrameFormat, error> {
    return put(tx_data, timeout).and_then([&]() { return get(timeout); });
  }

  auto wait_idle(std::chrono::milliseconds timeout)
      -> std::expected<void, error> {
    emb::chrono::timeout t(timeout);
    while (!tx_empty() || busy()) {
      if (t.expired()) return std::unexpected(error::timeout);
    }
    return {};
  }

  template<typename Fn>
  auto transaction(Fn&& fn) -> decltype(fn())
    requires(SlaveCount == 1) {
    select();
    auto result = fn();
    release();
    return result;
  }

  template<size_t Idx, typename Fn>
  auto transaction(Fn&& fn) -> decltype(fn()) {
    select<Idx>();
    auto result = fn();
    release<Idx>();
    return result;
  }

private:
  void clear_overrun() const {
    [[maybe_unused]] auto volatile d = REG.DATA;
    [[maybe_unused]] auto volatile s = REG.STS;
  }
};

} // namespace spi
} // namespace f4
} // namespace apm32
