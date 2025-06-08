#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/spi/spi.hpp>

namespace mcu {
namespace apm32 {
namespace spi {

internal::MosiPin::MosiPin(MosiPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output = gpio::Output::pushpull,
                          .speed = gpio::Speed::high,
                          .altfunc = conf.altfunc}} {}

internal::MisoPin::MisoPin(MisoPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output = gpio::Output::pushpull,
                          .speed = gpio::Speed::high,
                          .altfunc = conf.altfunc}} {}

internal::ClkPin::ClkPin(ClkPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output = gpio::Output::pushpull,
                          .speed = gpio::Speed::high,
                          .altfunc = conf.altfunc}} {}

internal::HwSsPin::HwSsPin(HwSsPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output = gpio::Output::pushpull,
                          .speed = gpio::Speed::medium,
                          .altfunc = conf.altfunc}} {}

internal::SwSsPin::SwSsPin(SwSsPinConfig const& conf)
    : gpio::DigitalOutput{{.port = conf.port,
                           .pin = conf.pin,
                           .pull = gpio::Pull::up,
                           .output = gpio::Output::pushpull,
                           .speed = gpio::Speed::medium,
                           .active_state = mcu::gpio::active_state::low}} {}

Module::Module(Peripheral peripheral,
               MosiPinConfig const& mosi_pin_conf,
               MisoPinConfig const& miso_pin_conf,
               ClkPinConfig const& clk_pin_conf,
               HwSsPinConfig const& ss_pin_conf,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{spi::regs[std::to_underlying(peripheral)]},
      mosi_pin_{mosi_pin_conf},
      miso_pin_(miso_pin_conf),
      clk_pin_{clk_pin_conf} {
  ss_pin_ = std::make_unique<internal::HwSsPin>(ss_pin_conf);
  enable_clk(peripheral);
  SPI_Config(regs_, const_cast<SPI_Config_T*>(&conf.hal_config));
  SPI_Enable(regs_);
}

Module::Module(Peripheral peripheral,
               MosiPinConfig const& mosi_pin_conf,
               MisoPinConfig const& miso_pin_conf,
               ClkPinConfig const& clk_pin_conf,
               std::initializer_list<SwSsPinConfig> ss_pin_confs,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{spi::regs[std::to_underlying(peripheral)]},
      mosi_pin_{mosi_pin_conf},
      miso_pin_(miso_pin_conf),
      clk_pin_{clk_pin_conf} {
  if (ss_pin_confs.size() != 0 && conf.hal_config.mode != SPI_MODE_MASTER) {
    fatal_error();
  }

  for (auto ss_pin_conf : ss_pin_confs) {
    ss_pins_.emplace_back(std::make_unique<internal::SwSsPin>(ss_pin_conf));
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
} // namespace apm32
} // namespace mcu

#endif
#endif
