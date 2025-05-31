#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_gpio.h>
#include <apm32f4xx_rcm.h>

#include <emblib/mcudef.hpp>
#include <mcudrv-apm32/f4/apm32f4.hpp>
#include <mcudrv-apm32/f4/system/system.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdint.h>
#include <utility>

namespace mcu {
namespace apm32 {
namespace gpio {

using PortRegs = GPIO_T;

constexpr size_t port_num{9};

enum class Port : size_t {
  gpioa,
  gpiob,
  gpioc,
  gpiod,
  gpioe,
  gpiof,
  gpiog,
  gpioh,
  gpioi
};

inline std::array<PortRegs*, port_num> const regs{
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

enum class Pin : uint16_t {
  pin0 = GPIO_PIN_0,
  pin1 = GPIO_PIN_1,
  pin2 = GPIO_PIN_2,
  pin3 = GPIO_PIN_3,
  pin4 = GPIO_PIN_4,
  pin5 = GPIO_PIN_5,
  pin6 = GPIO_PIN_6,
  pin7 = GPIO_PIN_7,
  pin8 = GPIO_PIN_8,
  pin9 = GPIO_PIN_9,
  pin10 = GPIO_PIN_10,
  pin11 = GPIO_PIN_11,
  pin12 = GPIO_PIN_12,
  pin13 = GPIO_PIN_13,
  pin14 = GPIO_PIN_14,
  pin15 = GPIO_PIN_15,
};

enum class Output : uint32_t {
  pushpull = GPIO_OTYPE_PP,
  opendrain = GPIO_OTYPE_OD
};

enum class Speed : uint32_t {
  low = GPIO_SPEED_2MHz,
  medium = GPIO_SPEED_25MHz,
  fast = GPIO_SPEED_50MHz,
  high = GPIO_SPEED_100MHz
};

enum class Pull : uint32_t {
  none = GPIO_PUPD_NOPULL,
  up = GPIO_PUPD_UP,
  down = GPIO_PUPD_DOWN
};
;

struct DigitalInputConfig {
  Port port;
  Pin pin;
  Pull pull;
  mcu::gpio::active_state active_state;
};

struct DigitalOutputConfig {
  Port port;
  Pin pin;
  Pull pull;
  Output output;
  Speed speed;
  mcu::gpio::active_state active_state;
};

struct AlternatePinConfig {
  Port port;
  Pin pin;
  Pull pull;
  Output output;
  Speed speed;
  GPIO_AF_T altfunc;
};

struct AnalogPinConfig {
  Port port;
  Pin pin;
};

namespace internal {

inline std::array<void (*)(void), port_num> clk_enable_funcs{
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOB); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOC); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOD); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOE); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOF); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOG); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOH); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOI); }};

class Pin {
private:
  static inline std::array<uint16_t, port_num> assigned_{};
  static inline std::array<bool, port_num> clk_enabled_{};
protected:
  Port const port_;
  uint16_t const pin_;
  PortRegs* const regs_;
private:
  Pin(Port port,
      GPIO_Config_T const& conf,
      std::optional<GPIO_AF_T> altfunc = std::nullopt);
protected:
  ~Pin();
  Pin(DigitalInputConfig const& conf);
  Pin(DigitalOutputConfig const& conf);
  Pin(AlternatePinConfig const& conf);
  Pin(AnalogPinConfig const& conf);
public:
  unsigned int pin_no() const { return __CLZ(__RBIT(pin_)); }

  uint16_t pin_bit() const { return static_cast<uint16_t>(pin_); }

  PortRegs const* regs() const { return regs_; }
};

} // namespace internal

class DigitalInput : public mcu::gpio::input_pin, public internal::Pin {
  // friend void ::EXTI0_IRQHandler();
  // friend void ::EXTI1_IRQHandler();
  // friend void ::EXTI2_IRQHandler();
  // friend void ::EXTI3_IRQHandler();
  // friend void ::EXTI4_IRQHandler();
  // friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
private:
  mcu::gpio::active_state const active_state_;
public:
  DigitalInput(DigitalInputConfig const& conf)
      : internal::Pin(conf), active_state_(conf.active_state) {}

  mcu::gpio::active_state active_state() const { return active_state_; }

  virtual unsigned int read_level() const override {
    if ((read_reg(regs_->IDATA) & pin_) != 0) {
      return 1;
    }
    return 0;
  }

  virtual mcu::gpio::pin_state read() const override {
    if (read_level() == std::to_underlying(active_state_)) {
      return mcu::gpio::pin_state::active;
    }
    return mcu::gpio::pin_state::inactive;
  }

  // clang-format off
    // TODO
    // private:
    //     IRQn_Type _irqn = NonMaskableInt_IRQn;	// use NonMaskableInt_IRQn as value for not initialized interrupt
    //     static inline std::array<void(*)(void), 16> on_interrupt = {
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //     };
    // public:
    //     void initialize_interrupt(void(*handler)(void), IrqPriority priority) {
    //         switch (_config.pin.Pin) {
    //         case GPIO_PIN_0:
    //             _irqn = EXTI0_IRQn;
    //             break;
    //         case GPIO_PIN_1:
    //             _irqn = EXTI1_IRQn;
    //             break;
    //         case GPIO_PIN_2:
    //             _irqn = EXTI2_IRQn;
    //             break;
    //         case GPIO_PIN_3:
    //             _irqn = EXTI3_IRQn;
    //             break;
    //         case GPIO_PIN_4:
    //             _irqn = EXTI4_IRQn;
    //             break;
    //         case GPIO_PIN_5: case GPIO_PIN_6: case GPIO_PIN_7: case GPIO_PIN_8: case GPIO_PIN_9:
    //             _irqn = EXTI9_5_IRQn;
    //             break;
    //         case GPIO_PIN_10: case GPIO_PIN_11: case GPIO_PIN_12: case GPIO_PIN_13: case GPIO_PIN_14: case GPIO_PIN_15:
    //             _irqn = EXTI15_10_IRQn;
    //             break;
    //         default:
    //             _irqn = NonMaskableInt_IRQn;
    //             return;
    //         }
    //         HAL_NVIC_SetPriority(_irqn, priority.get(), 0);
    //         on_interrupt[this->pin_no()] = handler;
    //     }

    //     void enable_interrupts() {
    //         if (_irqn != NonMaskableInt_IRQn) {
    //             HAL_NVIC_EnableIRQ(_irqn);
    //         }
    //     }

    //     void disable_interrupts() {
    //         if (_irqn != NonMaskableInt_IRQn) {
    //             HAL_NVIC_EnableIRQ(_irqn);
    //         }
    //     }
  // clang-format on
};

class DigitalOutput : public mcu::gpio::output_pin, public internal::Pin {
private:
  mcu::gpio::active_state const active_state_;
public:
  DigitalOutput(
      DigitalOutputConfig const& conf,
      mcu::gpio::pin_state init_state = mcu::gpio::pin_state::inactive)
      : internal::Pin(conf), active_state_(conf.active_state) {
    set(init_state);
  }

  mcu::gpio::active_state active_state() const { return active_state_; }

  virtual unsigned int read_level() const override {
    if ((read_reg(regs_->IDATA) & pin_) != 0) {
      return 1;
    }
    return 0;
  }

  virtual void set_level(unsigned int level) override {
    if (level != 0) {
      write_reg(regs_->BSCL, pin_);
    } else {
      write_reg(regs_->BSCH, pin_);
    }
  }

  virtual mcu::gpio::pin_state read() const override {
    if (read_level() == std::to_underlying(active_state_)) {
      return mcu::gpio::pin_state::active;
    }
    return mcu::gpio::pin_state::inactive;
  }

  virtual void
  set(mcu::gpio::pin_state s = mcu::gpio::pin_state::active) override {
    if (s == mcu::gpio::pin_state::active) {
      set_level(std::to_underlying(active_state_));
    } else {
      set_level(1 - std::to_underlying(active_state_));
    }
  }

  virtual void reset() override { set(mcu::gpio::pin_state::inactive); }

  virtual void toggle() override {
    uint16_t const odr_reg{static_cast<uint16_t>(read_reg(regs_->ODATA))};
    write_reg<uint16_t>(regs_->BSCL, ~odr_reg & pin_);
    write_reg<uint16_t>(regs_->BSCH, odr_reg & pin_);
  }
};

class AlternatePin : public internal::Pin {
public:
  AlternatePin(AlternatePinConfig const& conf) : internal::Pin(conf) {}
};

class AnalogPin : public internal::Pin {
public:
  AnalogPin(AnalogPinConfig const& conf) : internal::Pin(conf) {}
};

enum class DurationLoggerMode {
  set_reset,
  toggle
};

enum class DurationLoggerChannel : unsigned int {
  channel0,
  channel1,
  channel2,
  channel3,
  channel4,
  channel5,
  channel6,
  channel7,
  channel8,
  channel9,
  channel10,
  channel11,
  channel12,
  channel13,
  channel14,
  channel15,
};

class DurationLogger {
private:
  static inline std::array<std::unique_ptr<DigitalOutput>, 16> pins_{};
  DigitalOutput* const pin_;
  DurationLoggerMode const mode_;
public:
  static void init_channel(DurationLoggerChannel channel, Port port, Pin pin) {
    pins_[std::to_underlying(channel)] = std::make_unique<DigitalOutput>(
        DigitalOutputConfig{.port = port,
                            .pin = pin,
                            .pull = Pull::none,
                            .output = Output::pushpull,
                            .speed = Speed::high,
                            .active_state = mcu::gpio::active_state::high});
  }

  DurationLogger(DurationLoggerChannel channel, DurationLoggerMode mode)
      : pin_(pins_[std::to_underlying(channel)].get()), mode_(mode) {
    if (!pin_) {
      return;
    }

    if (mode_ == DurationLoggerMode::set_reset) {
      pin_->set_level(1);
    } else {
      pin_->toggle();
      pin_->toggle();
    }
  }

  ~DurationLogger() {
    if (!pin_) {
      return;
    }

    if (mode_ == DurationLoggerMode::set_reset) {
      pin_->set_level(0);
    } else {
      pin_->toggle();
    }
  }
};

} // namespace gpio
} // namespace apm32
} // namespace mcu

#endif
#endif
