#pragma once

#include <apm32/f4/gpio/gpio_types.hpp>

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32f4xx_gpio.h>
#include <apm32f4xx_rcm.h>

#include <emb/gpio.hpp>

#include <array>
#include <memory>
#include <optional>
#include <utility>

namespace apm32 {
namespace f4 {
namespace gpio {

using port_registers = GPIO_T;

inline constexpr size_t port_count = 9;

inline std::array<port_registers*, port_count> const ports =
    {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

enum class port : uint32_t {
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

enum class pin : uint16_t {
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

enum class output_type : uint32_t {
  pushpull = GPIO_OTYPE_PP,
  opendrain = GPIO_OTYPE_OD
};

enum class speed : uint32_t {
  low = GPIO_SPEED_2MHz,
  medium = GPIO_SPEED_25MHz,
  fast = GPIO_SPEED_50MHz,
  high = GPIO_SPEED_100MHz
};

enum class pull : uint32_t {
  none = GPIO_PUPD_NOPULL,
  up = GPIO_PUPD_UP,
  down = GPIO_PUPD_DOWN
};
;

struct input_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
  apm32::f4::gpio::pull pull;
  emb::gpio::level active_level;
};

struct output_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
  apm32::f4::gpio::pull pull;
  apm32::f4::gpio::output_type output_type;
  apm32::f4::gpio::speed speed;
  emb::gpio::level active_level;
};

struct alternate_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
  apm32::f4::gpio::pull pull;
  apm32::f4::gpio::output_type output_type;
  apm32::f4::gpio::speed speed;
  GPIO_AF_T altfunc;
};

struct analog_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
};

namespace detail {

class pin_base {
protected:
  port const port_;
  uint16_t const pin_;
  port_registers* const regs_;
private:
  pin_base(
      port p,
      GPIO_Config_T conf,
      std::optional<GPIO_AF_T> altfunc = std::nullopt
  );
protected:
  ~pin_base();
  explicit pin_base(input_pin_config const& conf);
  explicit pin_base(output_pin_config const& conf);
  explicit pin_base(alternate_pin_config const& conf);
  explicit pin_base(analog_pin_config const& conf);
public:
  pin_base(pin_base const& other) = delete;
  pin_base& operator=(pin_base const& other) = delete;

  size_t pin_no() const {
    return __CLZ(__RBIT(pin_));
  }

  uint16_t pin_bit() const {
    return static_cast<uint16_t>(pin_);
  }

  port_registers const* regs() const {
    return regs_;
  }
private:
  static inline std::array<uint16_t, port_count> used_pins_{};
  static inline std::array<bool, port_count> is_clock_enabled_{};
  static std::array<void (*)(void), port_count> enable_port_clock_;
};

} // namespace detail

class input_pin : public detail::pin_base {
  // friend void ::EXTI0_IRQHandler();
  // friend void ::EXTI1_IRQHandler();
  // friend void ::EXTI2_IRQHandler();
  // friend void ::EXTI3_IRQHandler();
  // friend void ::EXTI4_IRQHandler();
  // friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
private:
  emb::gpio::level const active_level_;
public:
  explicit input_pin(input_pin_config const& conf)
      : detail::pin_base(conf), active_level_(conf.active_level) {}

  input_pin(input_pin const& other) = delete;
  input_pin& operator=(input_pin const& other) = delete;

  emb::gpio::level active_level() const {
    return active_level_;
  }

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

static_assert(emb::gpio::input<input_pin>);

class output_pin : public detail::pin_base {
private:
  emb::gpio::level const active_level_;
public:
  explicit output_pin(
      output_pin_config const& conf,
      emb::gpio::state init_state = emb::gpio::state::inactive
  )
      : detail::pin_base(conf), active_level_(conf.active_level) {
    set(init_state);
  }

  output_pin(output_pin const& other) = delete;
  output_pin& operator=(output_pin const& other) = delete;

  emb::gpio::level active_level() const {
    return active_level_;
  }

  emb::gpio::level read_level() const {
    if ((read_reg(regs_->IDATA) & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }

  void set_level(emb::gpio::level lvl) {
    if (lvl == emb::gpio::level::high) {
      write_reg(regs_->BSCL, pin_);
    } else {
      write_reg(regs_->BSCH, pin_);
    }
  }

  emb::gpio::state read() const {
    if (read_level() == active_level_) {
      return emb::gpio::state::active;
    }
    return emb::gpio::state::inactive;
  }

  void set(emb::gpio::state s = emb::gpio::state::active) {
    if (s == emb::gpio::state::active) {
      set_level(active_level_);
    } else {
      set_level(!active_level_);
    }
  }

  void reset() {
    set(emb::gpio::state::inactive);
  }

  void toggle() {
    uint16_t const out = static_cast<uint16_t>(read_reg(regs_->ODATA));
    write_reg<uint16_t>(regs_->BSCL, ~out & pin_);
    write_reg<uint16_t>(regs_->BSCH, out & pin_);
  }
};

class alternate_pin : public detail::pin_base {
public:
  explicit alternate_pin(alternate_pin_config const& conf)
      : detail::pin_base(conf) {}

  alternate_pin(alternate_pin const& other) = delete;
  alternate_pin& operator=(alternate_pin const& other) = delete;

  emb::gpio::level read_level() const {
    if ((read_reg(regs_->IDATA) & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }
};

class analog_pin : public detail::pin_base {
public:
  explicit analog_pin(analog_pin_config const& conf) : detail::pin_base(conf) {}

  analog_pin(analog_pin const& other) = delete;
  analog_pin& operator=(analog_pin const& other) = delete;
};

} // namespace gpio
} // namespace f4
} // namespace apm32
