#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/gpio/low_layer/gpio_pins.hpp>
#include <apm32/f4/gpio/low_layer/gpio_ports.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace apm32::f4::gpio {

namespace detail {

class pin_base {
protected:
  port const port_;
  std::uint16_t const pin_;
  port_registers* const regs_;
private:
  pin_base(
      port p,
      std::uint16_t pin_mask,
      std::uint32_t mode_val,
      std::uint32_t otype_val,
      std::uint32_t speed_val,
      std::uint32_t pupd_val,
      std::optional<std::uint32_t> altfunc = std::nullopt
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

  std::size_t pin_no() const {
    return __CLZ(__RBIT(pin_));
  }

  std::uint16_t pin_bit() const {
    return static_cast<std::uint16_t>(pin_);
  }

  port_registers const* regs() const {
    return regs_;
  }
private:
  static inline std::array<std::uint16_t, port_count> used_pins_{};
  static inline std::array<bool, port_count> is_clock_enabled_{};

  static inline constexpr std::array<std::uint32_t, port_count>
      port_clock_bits_ = {
          RCM_AHB1CLKEN_PAEN,
          RCM_AHB1CLKEN_PBEN,
          RCM_AHB1CLKEN_PCEN,
          RCM_AHB1CLKEN_PDEN,
          RCM_AHB1CLKEN_PEEN,
          RCM_AHB1CLKEN_PFEN,
          RCM_AHB1CLKEN_PGEN,
          RCM_AHB1CLKEN_PHEN,
          RCM_AHB1CLKEN_PIEN,
  };
};

} // namespace detail

} // namespace apm32::f4::gpio
