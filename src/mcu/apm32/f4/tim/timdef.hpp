#pragma once

#ifdef APM32F4XX

#include <apm32f4xx_tmr.h>

#include <mcu/apm32/f4/gpio.hpp>
#include <mcu/apm32/f4/system.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
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
  emb::gpio::level active_level;
};

namespace internal {

class ChPin : public gpio::AlternatePin {
public:
  ChPin(ChPinConfig const& conf)
      : gpio::AlternatePin{gpio::AlternateConfig{
            .port = conf.port,
            .pin = conf.pin,
            .pull = gpio::Pull::none,
            .output_type = gpio::OutputType::pushpull,
            .speed = gpio::Speed::fast,
            .altfunc = conf.altfunc}} {}
};

class BkinPin : public gpio::AlternatePin {
private:
  emb::gpio::level const active_level_;
public:
  BkinPin(BkinPinConfig const& conf)
      : gpio::AlternatePin{gpio::AlternateConfig{
            .port = conf.port,
            .pin = conf.pin,
            .pull = conf.pull,
            .output_type = gpio::OutputType::pushpull,
            .speed = gpio::Speed::fast,
            .altfunc = conf.altfunc}},
        active_level_{conf.active_level} {}

  emb::gpio::level active_level() const { return active_level_; }

  emb::gpio::level read_level() const {
    if ((read_reg(regs_->IDATA) & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }

  emb::gpio::state read() const {
    if (read_level() == active_level_) {
      return emb::gpio::state::active;
    }
    return emb::gpio::state::inactive;
  }
};

} // namespace internal

} // namespace tim
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
