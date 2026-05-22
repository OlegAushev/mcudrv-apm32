#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/gpio/gpio.hpp>

#include <emb/gpio.hpp>

#include <cstdint>

namespace apm32::f4::gpio {

class output_pin : public detail::pin_base {
private:
  emb::gpio::polarity const polarity_;
public:
  explicit output_pin(
      output_pin_config const& conf,
      emb::gpio::state init_state = emb::gpio::state::inactive
  )
      : detail::pin_base(conf), polarity_(conf.polarity) {
    set(init_state);
  }

  output_pin(output_pin const& other) = delete;
  output_pin& operator=(output_pin const& other) = delete;
  output_pin(output_pin&& other) = delete;
  output_pin& operator=(output_pin&& other) = delete;

  emb::gpio::polarity polarity() const {
    return polarity_;
  }

  emb::gpio::level read_level() const {
    if ((regs_->IDATA & pin_) != 0) {
      return emb::gpio::level::high;
    }
    return emb::gpio::level::low;
  }

  void set_level(emb::gpio::level lvl) {
    if (lvl == emb::gpio::level::high) {
      regs_->BSC = std::uint32_t(pin_);
    } else {
      regs_->BSC = std::uint32_t(pin_) << 16;
    }
  }

  emb::gpio::state read() const {
    return emb::gpio::to_state(read_level(), polarity_);
  }

  void set(emb::gpio::state s = emb::gpio::state::active) {
    set_level(emb::gpio::to_level(s, polarity_));
  }

  void reset() {
    set(emb::gpio::state::inactive);
  }

  void toggle() {
    std::uint16_t const out = static_cast<std::uint16_t>(regs_->ODATA);
    regs_->BSC = std::uint32_t(~out & pin_) | (std::uint32_t(out & pin_) << 16);
  }
};

} // namespace apm32::f4::gpio
