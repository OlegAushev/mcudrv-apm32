#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_gpio.h>
#include <apm32f4xx_rcm.h>

#include <emb/gpio.hpp>
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
inline namespace apm32 {
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

enum class OutputType : uint32_t {
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

struct InputConfig {
  Port port;
  Pin pin;
  Pull pull;
  emb::gpio::level active_level;
};

struct OutputConfig {
  Port port;
  Pin pin;
  Pull pull;
  OutputType output_type;
  Speed speed;
  emb::gpio::level active_level;
};

struct AlternateConfig {
  Port port;
  Pin pin;
  Pull pull;
  OutputType output_type;
  Speed speed;
  GPIO_AF_T altfunc;
};

struct AnalogConfig {
  Port port;
  Pin pin;
};

namespace internal {

class Pin {
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
  explicit Pin(InputConfig const& conf);
  explicit Pin(OutputConfig const& conf);
  explicit Pin(AlternateConfig const& conf);
  explicit Pin(AnalogConfig const& conf);
public:
  Pin(Pin const& other) = delete;
  Pin& operator=(Pin const& other) = delete;

  unsigned int pin_no() const { return __CLZ(__RBIT(pin_)); }

  uint16_t pin_bit() const { return static_cast<uint16_t>(pin_); }

  PortRegs const* regs() const { return regs_; }
private:
  static inline std::array<uint16_t, port_num> assigned_{};
  static inline std::array<bool, port_num> clk_enabled_{};
  static std::array<void (*)(void), port_num> enable_port_clk_;
};

} // namespace internal

class Input : public emb::gpio::input, public internal::Pin {
  // friend void ::EXTI0_IRQHandler();
  // friend void ::EXTI1_IRQHandler();
  // friend void ::EXTI2_IRQHandler();
  // friend void ::EXTI3_IRQHandler();
  // friend void ::EXTI4_IRQHandler();
  // friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
private:
  emb::gpio::level const active_level_;
public:
  explicit Input(InputConfig const& conf)
      : internal::Pin(conf), active_level_(conf.active_level) {}

  Input(Input const& other) = delete;
  Input& operator=(Input const& other) = delete;

  emb::gpio::level active_level() const { return active_level_; }

  virtual emb::gpio::level read_level() const override {
    if ((read_reg(regs_->IDATA) & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }

  virtual emb::gpio::state read() const override {
    if (read_level() == active_level_) {
      return emb::gpio::state::active;
    }
    return emb::gpio::state::inactive;
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

class Output : public emb::gpio::output, public internal::Pin {
private:
  emb::gpio::level const active_level_;
public:
  explicit Output(OutputConfig const& conf,
                  emb::gpio::state init_state = emb::gpio::state::inactive)
      : internal::Pin(conf), active_level_(conf.active_level) {
    set(init_state);
  }

  Output(Output const& other) = delete;
  Output& operator=(Output const& other) = delete;

  emb::gpio::level active_level() const { return active_level_; }

  virtual emb::gpio::level read_level() const override {
    if ((read_reg(regs_->IDATA) & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }

  virtual void set_level(emb::gpio::level lvl) override {
    if (lvl == emb::gpio::level::high) {
      write_reg(regs_->BSCL, pin_);
    } else {
      write_reg(regs_->BSCH, pin_);
    }
  }

  virtual emb::gpio::state read() const override {
    if (read_level() == active_level_) {
      return emb::gpio::state::active;
    }
    return emb::gpio::state::inactive;
  }

  virtual void set(emb::gpio::state s = emb::gpio::state::active) override {
    if (s == emb::gpio::state::active) {
      set_level(active_level_);
    } else {
      set_level(!active_level_);
    }
  }

  virtual void reset() override { set(emb::gpio::state::inactive); }

  virtual void toggle() override {
    uint16_t const odr_reg{static_cast<uint16_t>(read_reg(regs_->ODATA))};
    write_reg<uint16_t>(regs_->BSCL, ~odr_reg & pin_);
    write_reg<uint16_t>(regs_->BSCH, odr_reg & pin_);
  }
};

class AlternatePin : public internal::Pin {
public:
  explicit AlternatePin(AlternateConfig const& conf) : internal::Pin(conf) {}

  AlternatePin(AlternatePin const& other) = delete;
  AlternatePin& operator=(AlternatePin const& other) = delete;
};

class AnalogPin : public internal::Pin {
public:
  explicit AnalogPin(AnalogConfig const& conf) : internal::Pin(conf) {}

  AnalogPin(AnalogPin const& other) = delete;
  AnalogPin& operator=(AnalogPin const& other) = delete;
};

namespace util {

enum class LoggerMode {
  set_reset,
  toggle
};

enum class LoggerChannel : unsigned int {
  ch0,
  ch1,
  ch2,
  ch3,
  ch4,
  ch5,
  ch6,
  ch7,
  ch8,
  ch9,
  ch10,
  ch11,
  ch12,
  ch13,
  ch14,
  ch15,
};

class Logger {
private:
  static inline std::array<std::unique_ptr<Output>, 16> pins_{};
  Output* const pin_;
  LoggerMode const mode_;
public:
  static void init_channel(LoggerChannel ch, Port port, Pin pin) {
    pins_[std::to_underlying(ch)] = std::make_unique<Output>(
        OutputConfig{.port = port,
                     .pin = pin,
                     .pull = Pull::none,
                     .output_type = OutputType::pushpull,
                     .speed = Speed::high,
                     .active_level = emb::gpio::level::high});
  }

  Logger(LoggerChannel ch, LoggerMode mode)
      : pin_(pins_[std::to_underlying(ch)].get()), mode_(mode) {
    if (!pin_) {
      return;
    }

    if (mode_ == LoggerMode::set_reset) {
      pin_->set_level(emb::gpio::level::high);
    } else {
      pin_->toggle();
      __NOP();
      pin_->toggle();
    }
  }

  ~Logger() {
    if (!pin_) {
      return;
    }

    if (mode_ == LoggerMode::set_reset) {
      pin_->set_level(emb::gpio::level::low);
    } else {
      pin_->toggle();
    }
  }

  void mark() {
    if (!pin_) {
      return;
    }

    pin_->toggle();
    __NOP();
    pin_->toggle();
  }

  static std::optional<emb::gpio::level> read(LoggerChannel ch) {
    auto const pin{pins_[std::to_underlying(ch)].get()};

    if (!pin) {
      return std::nullopt;
    }

    return pin->read_level();
  }
};

} // namespace util

} // namespace gpio
} // namespace apm32
} // namespace mcu

#endif
#endif
