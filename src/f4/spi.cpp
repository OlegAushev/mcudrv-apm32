#include <apm32/f4/spi.hpp>

namespace apm32 {
namespace f4 {
namespace spi {

detail::mosi_pin::mosi_pin(mosi_pin_config const& conf)
    : gpio::alternate_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::high,
           .altfunc = conf.altfunc}
      } {}

detail::miso_pin::miso_pin(miso_pin_config const& conf)
    : gpio::alternate_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::high,
           .altfunc = conf.altfunc}
      } {}

detail::clk_pin::clk_pin(clk_pin_config const& conf)
    : gpio::alternate_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::high,
           .altfunc = conf.altfunc}
      } {}

detail::hardware_ss_pin::hardware_ss_pin(hardware_ss_pin_config const& conf)
    : gpio::alternate_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::medium,
           .altfunc = conf.altfunc}
      } {}

detail::software_ss_pin::software_ss_pin(software_ss_pin_config const& conf)
    : gpio::output_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::up,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::medium,
           .active_level = emb::gpio::level::low}
      } {}

peripheral::peripheral(
    peripheral_id id,
    mosi_pin_config const& mosi_pinconf,
    miso_pin_config const& miso_pinconf,
    clk_pin_config const& clk_pinconf,
    hardware_ss_pin_config const& ss_pinconf,
    config conf
)
    : peripheral_type(id),
      id_(id),
      regs_(peripherals[std::to_underlying(id)]),
      mosi_pin_(mosi_pinconf),
      miso_pin_(miso_pinconf),
      clk_pin_(clk_pinconf) {
  ss_pin_ = std::make_unique<detail::hardware_ss_pin>(ss_pinconf);
  enable_clock(id_);
  SPI_Config(regs_, &conf.hal_config);
  SPI_Enable(regs_);
}

peripheral::peripheral(
    peripheral_id id,
    mosi_pin_config const& mosi_pinconf,
    miso_pin_config const& miso_pinconf,
    clk_pin_config const& clk_pinconf,
    std::initializer_list<software_ss_pin_config> ss_pinconfs,
    config conf
)
    : peripheral_type(id),
      id_(id),
      regs_(peripherals[std::to_underlying(id)]),
      mosi_pin_(mosi_pinconf),
      miso_pin_(miso_pinconf),
      clk_pin_(clk_pinconf) {
  core::ensure(
      ss_pinconfs.size() != 0 || conf.hal_config.mode != SPI_MODE_MASTER
  );

  for (auto ss_pinconf : ss_pinconfs) {
    ss_pins_.emplace_back(
        std::make_unique<detail::software_ss_pin>(ss_pinconf)
    );
    ss_pins_.back()->reset();
  }

  enable_clock(id_);
  SPI_Config(regs_, &conf.hal_config);
  SPI_Enable(regs_);
}

void peripheral::configure_interrupts(
    std::initializer_list<interrupt_event> events,
    nvic::irq_priority priority
) {
  for (auto event : events) {
    switch (event) {
    case interrupt_event::txe:
      regs_->CTRL2_B.TXBEIEN = 1;
      break;
    case interrupt_event::rxne:
      regs_->CTRL2_B.RXBNEIEN = 1;
      break;
    case interrupt_event::err:
      regs_->CTRL2_B.ERRIEN = 1;
      break;
    }
  }
  set_irq_priority(detail::irq_numbers[std::to_underlying(id_)], priority);
}

void peripheral::enable_clock(peripheral_id id) {
  size_t spi_idx{std::to_underlying(id)};
  if (is_clock_enabled_[spi_idx]) {
    return;
  }

  detail::enable_clock[spi_idx]();
  is_clock_enabled_[spi_idx] = true;
}

} // namespace spi
} // namespace f4
} // namespace apm32
