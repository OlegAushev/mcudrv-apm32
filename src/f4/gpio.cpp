#include <apm32/f4/gpio.hpp>

#include <apm32/f4/core.hpp>

namespace apm32 {
namespace f4 {
namespace gpio {

std::array<void (*)(void), port_count> detail::pin_base::enable_port_clock_ = {
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOB); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOC); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOD); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOE); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOF); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOG); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOH); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOI); }
};

detail::pin_base::pin_base(
    port p,
    GPIO_Config_T conf,
    std::optional<GPIO_AF_T> altfunc
)
    : port_(p), pin_(conf.pin), regs_(ports[std::to_underlying(p)]) {
  size_t const port_idx = std::to_underlying(port_);
  core::ensure(!(used_pins_[port_idx] & pin_));
  used_pins_[port_idx] |= uint16_t(pin_);

  if (!is_clock_enabled_[port_idx]) {
    enable_port_clock_[port_idx]();
    is_clock_enabled_[port_idx] = true;
  }

  if (conf.mode == GPIO_MODE_AF && altfunc.has_value()) {
    GPIO_ConfigPinAF(
        regs_,
        static_cast<GPIO_PIN_SOURCE_T>(bit_position(pin_)),
        altfunc.value()
    );
  }

  GPIO_Config(regs_, &conf);
}

detail::pin_base::~pin_base() {
  size_t const port_idx = std::to_underlying(port_);
  used_pins_[port_idx] &= ~uint16_t(pin_);
}

detail::pin_base::pin_base(input_pin_config const& conf)
    : pin_base(
          conf.port,
          GPIO_Config_T{
              .pin = std::to_underlying(conf.pin),
              .mode = GPIO_MODE_IN,
              .speed = static_cast<GPIO_SPEED_T>(speed::low),
              .otype = static_cast<GPIO_OTYPE_T>(output_type::pushpull),
              .pupd = static_cast<GPIO_PUPD_T>(conf.pull)
          }
      ) {}

detail::pin_base::pin_base(output_pin_config const& conf)
    : pin_base(
          conf.port,
          GPIO_Config_T{
              .pin = std::to_underlying(conf.pin),
              .mode = GPIO_MODE_OUT,
              .speed = static_cast<GPIO_SPEED_T>(conf.speed),
              .otype = static_cast<GPIO_OTYPE_T>(conf.output_type),
              .pupd = static_cast<GPIO_PUPD_T>(conf.pull)
          }
      ) {}

detail::pin_base::pin_base(alternate_pin_config const& conf)
    : pin_base(
          conf.port,
          GPIO_Config_T{
              .pin = std::to_underlying(conf.pin),
              .mode = GPIO_MODE_AF,
              .speed = static_cast<GPIO_SPEED_T>(conf.speed),
              .otype = static_cast<GPIO_OTYPE_T>(conf.output_type),
              .pupd = static_cast<GPIO_PUPD_T>(conf.pull)
          },
          conf.altfunc
      ) {}

detail::pin_base::pin_base(analog_pin_config const& conf)
    : pin_base(
          conf.port,
          GPIO_Config_T{
              .pin = std::to_underlying(conf.pin),
              .mode = GPIO_MODE_AN,
              .speed = static_cast<GPIO_SPEED_T>(speed::low),
              .otype = static_cast<GPIO_OTYPE_T>(output_type::pushpull),
              .pupd = static_cast<GPIO_PUPD_T>(pull::none)
          }
      ) {}

} // namespace gpio
} // namespace f4
} // namespace apm32
