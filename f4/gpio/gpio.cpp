#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/gpio/gpio.hpp>

namespace mcu {
namespace apm32 {
namespace gpio {

std::array<void (*)(void), port_num> internal::Pin::enable_port_clk_{
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOB); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOC); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOD); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOE); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOF); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOG); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOH); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOI); }};

internal::Pin::Pin(Port port,
                   GPIO_Config_T const& conf,
                   std::optional<GPIO_AF_T> altfunc)
    : port_(port), pin_(conf.pin), regs_{gpio::regs[std::to_underlying(port)]} {
  size_t const port_idx{std::to_underlying(port_)};

  if (assigned_[port_idx] & pin_) {
    fatal_error();
  }
  assigned_[port_idx] |= uint16_t(pin_);

  if (!clk_enabled_[port_idx]) {
    enable_port_clk_[port_idx]();
    clk_enabled_[port_idx] = true;
  }

  if (conf.mode == GPIO_MODE_AF && altfunc.has_value()) {
    GPIO_ConfigPinAF(regs_,
                     static_cast<GPIO_PIN_SOURCE_T>(bit_position(pin_)),
                     altfunc.value());
  }

  GPIO_Config(regs_, const_cast<GPIO_Config_T*>(&conf));
}

internal::Pin::~Pin() {
  size_t const port_idx{std::to_underlying(port_)};
  assigned_[port_idx] &= ~uint16_t(pin_);
}

internal::Pin::Pin(DigitalInputConfig const& conf)
    : Pin{conf.port,
          GPIO_Config_T{.pin = std::to_underlying(conf.pin),
                        .mode = GPIO_MODE_IN,
                        .speed = static_cast<GPIO_SPEED_T>(Speed::low),
                        .otype = static_cast<GPIO_OTYPE_T>(Output::pushpull),
                        .pupd = static_cast<GPIO_PUPD_T>(conf.pull)}} {}

internal::Pin::Pin(DigitalOutputConfig const& conf)
    : Pin{conf.port,
          GPIO_Config_T{.pin = std::to_underlying(conf.pin),
                        .mode = GPIO_MODE_OUT,
                        .speed = static_cast<GPIO_SPEED_T>(conf.speed),
                        .otype = static_cast<GPIO_OTYPE_T>(conf.output),
                        .pupd = static_cast<GPIO_PUPD_T>(conf.pull)}} {}

internal::Pin::Pin(AlternatePinConfig const& conf)
    : Pin{conf.port,
          GPIO_Config_T{.pin = std::to_underlying(conf.pin),
                        .mode = GPIO_MODE_AF,
                        .speed = static_cast<GPIO_SPEED_T>(conf.speed),
                        .otype = static_cast<GPIO_OTYPE_T>(conf.output),
                        .pupd = static_cast<GPIO_PUPD_T>(conf.pull)},
          conf.altfunc} {}

internal::Pin::Pin(AnalogPinConfig const& conf)
    : Pin{conf.port,
          GPIO_Config_T{.pin = std::to_underlying(conf.pin),
                        .mode = GPIO_MODE_AN,
                        .speed = static_cast<GPIO_SPEED_T>(Speed::low),
                        .otype = static_cast<GPIO_OTYPE_T>(Output::pushpull),
                        .pupd = static_cast<GPIO_PUPD_T>(Pull::none)}} {}

} // namespace gpio
} // namespace apm32
} // namespace mcu

#endif
#endif
