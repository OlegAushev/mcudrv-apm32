#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_tmr.h>

#include <mcudrv-apm32/f4/apm32f4.hpp>
#include <mcudrv-apm32/f4/gpio/gpio.hpp>
#include <mcudrv-apm32/f4/system/system.hpp>

namespace mcu {
namespace apm32 {
namespace tim {

enum class OpMode {
    inactive,
    timebase,
    input_capture,
    output_compare,
    pwm_generation,
    one_pulse
};

enum class CountDir { up, down };

struct ChPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};

class ChPin {
public:
    ChPin(const ChPinConfig& config) {
        gpio::AlternatePin({.port = config.port,
                            .pin = config.pin,
                            .config = {.pin{},
                                       .mode = GPIO_MODE_AF,
                                       .speed = GPIO_SPEED_50MHz,
                                       .otype = GPIO_OTYPE_PP,
                                       .pupd = GPIO_PUPD_NOPULL},
                            .altfunc = config.altfunc,
                            .active_state = mcu::gpio::active_state::high});
    }
};

struct BkinPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_PUPD_T pull;
    GPIO_AF_T altfunc;
};

class BkinPin {
public:
    BkinPin(const BkinPinConfig& config) {
        gpio::AlternatePin({.port = config.port,
                            .pin = config.pin,
                            .config = {.pin{},
                                       .mode = GPIO_MODE_AF,
                                       .speed = GPIO_SPEED_50MHz,
                                       .otype = GPIO_OTYPE_PP,
                                       .pupd = config.pull},
                            .altfunc = config.altfunc,
                            .active_state = mcu::gpio::active_state::high});
    }
};

} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
