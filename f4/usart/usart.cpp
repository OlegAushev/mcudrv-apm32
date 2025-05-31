#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/usart/usart.hpp>

namespace mcu {
namespace apm32 {
namespace usart {

Module::Module(Peripheral peripheral,
               RxPinConfig const& rx_pin_conf,
               TxPinConfig const& tx_pin_conf,
               Config const& config)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{impl::regs[std::to_underlying(peripheral)]},
      rx_pin_{gpio::AlternatePinConfig{.port = rx_pin_conf.port,
                                       .pin = rx_pin_conf.pin,
                                       .pull = gpio::Pull::none,
                                       .output = gpio::Output::pushpull,
                                       .speed = gpio::Speed::fast,
                                       .altfunc = rx_pin_conf.altfunc}},
      tx_pin_{gpio::AlternatePinConfig{.port = tx_pin_conf.port,
                                       .pin = tx_pin_conf.pin,
                                       .pull = gpio::Pull::none,
                                       .output = gpio::Output::pushpull,
                                       .speed = gpio::Speed::fast,
                                       .altfunc = tx_pin_conf.altfunc}} {
  enable_clk(peripheral);
  USART_Config(regs_, const_cast<USART_Config_T*>(&config.hal_config));
  USART_Enable(regs_);
}

void Module::enable_clk(Peripheral peripheral) {
  size_t usart_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[usart_idx]) {
    return;
  }

  impl::clk_enable_funcs[usart_idx]();
  clk_enabled_[usart_idx] = true;
}

} // namespace usart
} // namespace apm32
} // namespace mcu

#endif
#endif
