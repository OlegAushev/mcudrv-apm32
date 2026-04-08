#if 0
#include <apm32/f4/spi.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace spi {

detail::mosi_pin::mosi_pin(mosi_pin_config const& conf)
    : gpio::alternate_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::very_high,
           .altfunc = conf.altfunc}
      } {}

detail::miso_pin::miso_pin(miso_pin_config const& conf)
    : gpio::alternate_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::very_high,
           .altfunc = conf.altfunc}
      } {}

detail::clk_pin::clk_pin(clk_pin_config const& conf)
    : gpio::alternate_pin{
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::very_high,
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
  regs_->CTRL1 = conf.ctrl1();
  emb::mmio::set(regs_->CTRL1, SPI_CTRL1_SPIEN);
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
      ss_pinconfs.size() != 0 || conf.mode != mode::master
  );

  for (auto ss_pinconf : ss_pinconfs) {
    ss_pins_.emplace_back(
        std::make_unique<detail::software_ss_pin>(ss_pinconf)
    );
    ss_pins_.back()->reset();
  }

  enable_clock(id_);
  regs_->CTRL1 = conf.ctrl1();
  emb::mmio::set(regs_->CTRL1, SPI_CTRL1_SPIEN);
}

void peripheral::configure_interrupts(
    std::initializer_list<interrupt_event> events,
    nvic::irq_priority priority
) {
  for (auto event : events) {
    switch (event) {
    case interrupt_event::txe:
      emb::mmio::set(regs_->CTRL2, SPI_CTRL2_TXBEIEN);
      break;
    case interrupt_event::rxne:
      emb::mmio::set(regs_->CTRL2, SPI_CTRL2_RXBNEIEN);
      break;
    case interrupt_event::err:
      emb::mmio::set(regs_->CTRL2, SPI_CTRL2_ERRIEN);
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

  // SPI1 is on APB2, SPI2/SPI3 are on APB1
  if (id == peripheral_id::spi1) {
    emb::mmio::set(RCM->APB2CLKEN, detail::clock_bits[spi_idx]);
  } else {
    emb::mmio::set(RCM->APB1CLKEN, detail::clock_bits[spi_idx]);
  }
  is_clock_enabled_[spi_idx] = true;
}

} // namespace spi
} // namespace f4
} // namespace apm32
#endif