#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/usart/usart.hpp>

namespace mcu {
namespace apm32 {
namespace usart {

Module::Module(Peripheral peripheral,
               const RxPinConfig& rx_pin_config,
               const TxPinConfig& tx_pin_config,
               Config config)
        : emb::singleton_array<Module, peripheral_count>(
                  this, std::to_underlying(peripheral)),
          _peripheral(peripheral) {
    _rx_pin.init({.port = rx_pin_config.port,
                  .pin = rx_pin_config.pin,
                  .config = {.pin{},
                             .mode = GPIO_MODE_AF,
                             .speed = GPIO_SPEED_50MHz,
                             .otype = GPIO_OTYPE_PP,
                             .pupd = GPIO_PUPD_NOPULL},
                  .altfunc = rx_pin_config.altfunc,
                  .active_state = mcu::gpio::active_state::high});

    _tx_pin.init({.port = tx_pin_config.port,
                  .pin = tx_pin_config.pin,
                  .config = {.pin{},
                             .mode = GPIO_MODE_AF,
                             .speed = GPIO_SPEED_50MHz,
                             .otype = GPIO_OTYPE_PP,
                             .pupd = GPIO_PUPD_NOPULL},
                  .altfunc = tx_pin_config.altfunc,
                  .active_state = mcu::gpio::active_state::high});

    _enable_clk(peripheral);
    _reg = impl::usart_instances[std::to_underlying(peripheral)];

    USART_Config(_reg, &config.hal_config);
    USART_Enable(_reg);
}

void Module::_enable_clk(Peripheral peripheral) {
    size_t usart_idx = std::to_underlying(peripheral);
    if (_clk_enabled[usart_idx]) {
        return;
    }

    impl::usart_clk_enable_funcs[usart_idx]();
    _clk_enabled[usart_idx] = true;
}

} // namespace usart
} // namespace apm32
} // namespace mcu

#endif
#endif
