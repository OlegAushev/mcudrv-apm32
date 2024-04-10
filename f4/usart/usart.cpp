#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/usart/usart.h>


namespace mcu {
namespace usart {


Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, Config config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
{
    _rx_pin.initialize({
        .port = rx_pin_config.port,
        .pin = {
            .pin = rx_pin_config.pin,
            .mode = GPIO_MODE_AF,
            .speed = GPIO_SPEED_50MHz,
            .otype = GPIO_OTYPE_PP,
            .pupd = GPIO_PUPD_NOPULL
        },
        .altfunc = rx_pin_config.altfunc,
        .actstate = emb::gpio::active_pin_state::high
    });

    _tx_pin.initialize({
        .port = tx_pin_config.port,
        .pin = {
            .pin = tx_pin_config.pin,
            .mode = GPIO_MODE_AF,
            .speed = GPIO_SPEED_50MHz,
            .otype = GPIO_OTYPE_PP,
            .pupd = GPIO_PUPD_NOPULL
        },
        .altfunc = tx_pin_config.altfunc,
        .actstate = emb::gpio::active_pin_state::high
    });

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
} // namespace mcu


#endif
#endif
