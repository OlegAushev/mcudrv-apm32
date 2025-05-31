#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/spi/spi.hpp>

namespace mcu {
namespace apm32 {
namespace spi {

Module::Module(Peripheral peripheral,
               MosiPinConfig const& mosi_pin_conf,
               MisoPinConfig const& miso_pin_conf,
               ClkPinConfig const& clk_pin_conf,
               HwCsPinConfig const& cs_pin_conf,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{detail::regs[std::to_underlying(peripheral)]} {
  init_mosi_miso_clk(mosi_pin_conf, miso_pin_conf, clk_pin_conf);

  cs_pin_ = std::make_unique<gpio::AlternatePin>(
      gpio::AlternatePinConfig{.port = cs_pin_conf.port,
                               .pin = cs_pin_conf.pin,
                               .pull = gpio::Pull::none,
                               .output = gpio::Output::pushpull,
                               .speed = gpio::Speed::medium,
                               .altfunc = cs_pin_conf.altfunc});
  enable_clk(peripheral);
  SPI_Config(regs_, const_cast<SPI_Config_T*>(&conf.hal_config));
  SPI_Enable(regs_);
}

Module::Module(Peripheral peripheral,
               MosiPinConfig const& mosi_pin_conf,
               MisoPinConfig const& miso_pin_conf,
               ClkPinConfig const& clk_pin_conf,
               std::initializer_list<SwCsPinConfig> cs_pin_confs,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{detail::regs[std::to_underlying(peripheral)]} {
  if (cs_pin_confs.size() != 0 && conf.hal_config.mode != SPI_MODE_MASTER) {
    fatal_error();
  }

  init_mosi_miso_clk(mosi_pin_conf, miso_pin_conf, clk_pin_conf);

  for (auto pinconf : cs_pin_confs) {
    cs_pins_.emplace_back(
        std::make_unique<gpio::DigitalOutput>(gpio::DigitalOutputConfig{
            .port = pinconf.port,
            .pin = pinconf.pin,
            .pull = gpio::Pull::up,
            .output = gpio::Output::pushpull,
            .speed = gpio::Speed::medium,
            .active_state = mcu::gpio::active_state::low}));
    cs_pins_.back()->reset();
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
  set_irq_priority(detail::irqn[std::to_underlying(peripheral_)], priority);
}

void Module::enable_clk(Peripheral peripheral) {
  size_t spi_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[spi_idx]) {
    return;
  }

  detail::clk_enable_funcs[spi_idx]();
  clk_enabled_[spi_idx] = true;
}

void Module::init_mosi_miso_clk(MosiPinConfig const& mosi_pin_conf,
                                MisoPinConfig const& miso_pin_conf,
                                ClkPinConfig const& clk_pin_conf) {
  mosi_pin_ = std::make_unique<gpio::AlternatePin>(
      gpio::AlternatePinConfig{.port = mosi_pin_conf.port,
                               .pin = mosi_pin_conf.pin,
                               .pull = gpio::Pull::none,
                               .output = gpio::Output::pushpull,
                               .speed = gpio::Speed::high,
                               .altfunc = mosi_pin_conf.altfunc});
  miso_pin_ = std::make_unique<gpio::AlternatePin>(
      gpio::AlternatePinConfig{.port = miso_pin_conf.port,
                               .pin = miso_pin_conf.pin,
                               .pull = gpio::Pull::none,
                               .output = gpio::Output::pushpull,
                               .speed = gpio::Speed::high,
                               .altfunc = miso_pin_conf.altfunc});
  clk_pin_ = std::make_unique<gpio::AlternatePin>(
      gpio::AlternatePinConfig{.port = clk_pin_conf.port,
                               .pin = clk_pin_conf.pin,
                               .pull = gpio::Pull::none,
                               .output = gpio::Output::pushpull,
                               .speed = gpio::Speed::high,
                               .altfunc = clk_pin_conf.altfunc});
}

} // namespace spi
} // namespace apm32
} // namespace mcu

#endif
#endif
