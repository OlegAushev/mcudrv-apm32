#ifdef APM32F4XX

#include <mcu/apm32/f4/spi.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace spi {

internal::MosiPin::MosiPin(MosiPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::pushpull,
                          .speed = gpio::Speed::high,
                          .altfunc = conf.altfunc}} {}

internal::MisoPin::MisoPin(MisoPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::pushpull,
                          .speed = gpio::Speed::high,
                          .altfunc = conf.altfunc}} {}

internal::ClkPin::ClkPin(ClkPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::pushpull,
                          .speed = gpio::Speed::high,
                          .altfunc = conf.altfunc}} {}

internal::HwSsPin::HwSsPin(HwSsPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::pushpull,
                          .speed = gpio::Speed::medium,
                          .altfunc = conf.altfunc}} {}

internal::SwSsPin::SwSsPin(SwSsPinConfig const& conf)
    : gpio::Output{{.port = conf.port,
                    .pin = conf.pin,
                    .pull = gpio::Pull::up,
                    .output_type = gpio::OutputType::pushpull,
                    .speed = gpio::Speed::medium,
                    .active_level = emb::gpio::level::low}} {}

Module::Module(Peripheral peripheral,
               MosiPinConfig const& mosi_pinconf,
               MisoPinConfig const& miso_pinconf,
               ClkPinConfig const& clk_pinconf,
               HwSsPinConfig const& ss_pinconf,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{spi::regs[std::to_underlying(peripheral)]},
      mosi_pin_{mosi_pinconf},
      miso_pin_(miso_pinconf),
      clk_pin_{clk_pinconf} {
  ss_pin_ = std::make_unique<internal::HwSsPin>(ss_pinconf);
  enable_clk(peripheral);
  SPI_Config(regs_, const_cast<SPI_Config_T*>(&conf.hal_config));
  SPI_Enable(regs_);
}

Module::Module(Peripheral peripheral,
               MosiPinConfig const& mosi_pinconf,
               MisoPinConfig const& miso_pinconf,
               ClkPinConfig const& clk_pinconf,
               std::initializer_list<SwSsPinConfig> ss_pinconfs,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{spi::regs[std::to_underlying(peripheral)]},
      mosi_pin_{mosi_pinconf},
      miso_pin_(miso_pinconf),
      clk_pin_{clk_pinconf} {
  if (ss_pinconfs.size() != 0 && conf.hal_config.mode != SPI_MODE_MASTER) {
    fatal_error();
  }

  for (auto ss_pinconf : ss_pinconfs) {
    ss_pins_.emplace_back(std::make_unique<internal::SwSsPin>(ss_pinconf));
    ss_pins_.back()->reset();
  }

  enable_clk(peripheral);
  SPI_Config(regs_, const_cast<SPI_Config_T*>(&conf.hal_config));
  SPI_Enable(regs_);
}

void Module::init_interrupts(std::initializer_list<InterruptEvent> events,
                             IrqPriority priority) {
  for (auto event : events) {
    switch (event) {
    case InterruptEvent::txe:
      regs_->CTRL2_B.TXBEIEN = 1;
      break;
    case InterruptEvent::rxne:
      regs_->CTRL2_B.RXBNEIEN = 1;
      break;
    case InterruptEvent::err:
      regs_->CTRL2_B.ERRIEN = 1;
      break;
    }
  }
  set_irq_priority(internal::irqn[std::to_underlying(peripheral_)], priority);
}

void Module::enable_clk(Peripheral peripheral) {
  size_t spi_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[spi_idx]) {
    return;
  }

  enable_clk_[spi_idx]();
  clk_enabled_[spi_idx] = true;
}

} // namespace spi
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
