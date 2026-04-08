#pragma once

#include <apm32/f4/gpio/gpio_types.hpp>

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <emb/gpio.hpp>
#include <emb/mmio.hpp>

#include <array>
#include <memory>
#include <optional>
#include <utility>

namespace apm32 {
namespace f4 {
namespace gpio {

using port_registers = GPIO_TypeDef;

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
  pin0 = (1u << 0),
  pin1 = (1u << 1),
  pin2 = (1u << 2),
  pin3 = (1u << 3),
  pin4 = (1u << 4),
  pin5 = (1u << 5),
  pin6 = (1u << 6),
  pin7 = (1u << 7),
  pin8 = (1u << 8),
  pin9 = (1u << 9),
  pin10 = (1u << 10),
  pin11 = (1u << 11),
  pin12 = (1u << 12),
  pin13 = (1u << 13),
  pin14 = (1u << 14),
  pin15 = (1u << 15),
};

enum class output_type : uint32_t {
  pushpull = 0b0,
  opendrain = 0b1
};

enum class speed : uint32_t {
  low = 0b00,
  medium = 0b01,
  // high = 0b10,
  very_high = 0b11
};

enum class pull : uint32_t {
  none = 0b00,
  up = 0b01,
  down = 0b10
};

namespace mode {
inline constexpr uint32_t input = 0b00;
inline constexpr uint32_t output = 0b01;
inline constexpr uint32_t alternate = 0b10;
inline constexpr uint32_t analog = 0b11;
} // namespace mode

namespace altfunc {
inline constexpr uint32_t tmr1 = 1;
inline constexpr uint32_t tmr2 = 1;
inline constexpr uint32_t tmr3 = 2;
inline constexpr uint32_t tmr4 = 2;
inline constexpr uint32_t tmr5 = 2;
inline constexpr uint32_t tmr8 = 3;
inline constexpr uint32_t tmr9 = 3;
inline constexpr uint32_t tmr10 = 3;
inline constexpr uint32_t tmr11 = 3;
inline constexpr uint32_t i2c1 = 4;
inline constexpr uint32_t i2c2 = 4;
inline constexpr uint32_t i2c3 = 4;
inline constexpr uint32_t spi1 = 5;
inline constexpr uint32_t spi2 = 5;
inline constexpr uint32_t spi3 = 6;
inline constexpr uint32_t usart1 = 7;
inline constexpr uint32_t usart2 = 7;
inline constexpr uint32_t usart3 = 7;
inline constexpr uint32_t uart4 = 8;
inline constexpr uint32_t uart5 = 8;
inline constexpr uint32_t usart6 = 8;
inline constexpr uint32_t can1 = 9;
inline constexpr uint32_t can2 = 9;
inline constexpr uint32_t tmr12 = 9;
inline constexpr uint32_t tmr13 = 9;
inline constexpr uint32_t tmr14 = 9;
} // namespace altfunc

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
  uint32_t altfunc;
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
      uint16_t pin_mask,
      uint32_t mode_val,
      uint32_t otype_val,
      uint32_t speed_val,
      uint32_t pupd_val,
      std::optional<uint32_t> altfunc = std::nullopt
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

  static inline constexpr std::array<uint32_t, port_count> port_clock_bits_ = {
      RCM_AHB1CLKEN_PAEN, RCM_AHB1CLKEN_PBEN, RCM_AHB1CLKEN_PCEN,
      RCM_AHB1CLKEN_PDEN, RCM_AHB1CLKEN_PEEN, RCM_AHB1CLKEN_PFEN,
      RCM_AHB1CLKEN_PGEN, RCM_AHB1CLKEN_PHEN, RCM_AHB1CLKEN_PIEN,
  };
};

} // namespace detail

class input_pin : public detail::pin_base {
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
    if ((regs_->IDATA & pin_) != 0) {
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
    if ((regs_->IDATA & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }

  void set_level(emb::gpio::level lvl) {
    if (lvl == emb::gpio::level::high) {
      regs_->BSC = uint32_t(pin_);
    } else {
      regs_->BSC = uint32_t(pin_) << 16;
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
    uint16_t const out = static_cast<uint16_t>(regs_->ODATA);
    regs_->BSC = uint32_t(~out & pin_) | (uint32_t(out & pin_) << 16);
  }
};

class alternate_pin : public detail::pin_base {
public:
  explicit alternate_pin(alternate_pin_config const& conf)
      : detail::pin_base(conf) {}

  alternate_pin(alternate_pin const& other) = delete;
  alternate_pin& operator=(alternate_pin const& other) = delete;

  emb::gpio::level read_level() const {
    if ((regs_->IDATA & pin_) != 0) {
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
