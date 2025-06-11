#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/usart/usart.hpp>

namespace mcu {
namespace apm32 {
namespace usart {

internal::RxPin::RxPin(RxPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output = gpio::Output::pushpull,
                          .speed = gpio::Speed::fast,
                          .altfunc = conf.altfunc}} {}

internal::TxPin::TxPin(TxPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output = gpio::Output::pushpull,
                          .speed = gpio::Speed::fast,
                          .altfunc = conf.altfunc}} {}

Module::Module(Peripheral peripheral,
               RxPinConfig const& rx_pinconf,
               TxPinConfig const& tx_pinconf,
               Config const& config)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{usart::regs[std::to_underlying(peripheral)]},
      rx_pin_{rx_pinconf},
      tx_pin_{tx_pinconf} {
  enable_clk(peripheral);
  USART_Config(regs_, const_cast<USART_Config_T*>(&config.hal_config));
  USART_Enable(regs_);
}

void Module::enable_clk(Peripheral peripheral) {
  size_t usart_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[usart_idx]) {
    return;
  }

  enable_clk_[usart_idx]();
  clk_enabled_[usart_idx] = true;
}

} // namespace usart
} // namespace apm32
} // namespace mcu

#endif
#endif
