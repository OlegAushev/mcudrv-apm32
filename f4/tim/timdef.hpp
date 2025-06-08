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

enum class CountDir {
  up,
  down
};

struct ChPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct BkinPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  gpio::Pull pull;
  GPIO_AF_T altfunc;
};

namespace internal {

class ChPin : public gpio::AlternatePin {
public:
  ChPin(ChPinConfig const& conf)
      : gpio::AlternatePin{{.port = conf.port,
                            .pin = conf.pin,
                            .pull = gpio::Pull::none,
                            .output = gpio::Output::pushpull,
                            .speed = gpio::Speed::fast,
                            .altfunc = conf.altfunc}} {}
};

class BkinPin : public gpio::AlternatePin {
public:
  BkinPin(BkinPinConfig const& conf)
      : gpio::AlternatePin{{.port = conf.port,
                            .pin = conf.pin,
                            .pull = conf.pull,
                            .output = gpio::Output::pushpull,
                            .speed = gpio::Speed::fast,
                            .altfunc = conf.altfunc}} {}
};

} // namespace internal

} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
