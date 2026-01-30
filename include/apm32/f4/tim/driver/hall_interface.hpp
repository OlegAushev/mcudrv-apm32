#pragma once

#include <apm32/f4/tim/timer_instances.hpp>
#include <apm32/f4/tim/timer_types.hpp>
#include <apm32/f4/tim/timer_utils.hpp>

#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_tmr.h>

#include <emb/mmio.hpp>

#include <limits>
#include <optional>

namespace apm32 {
namespace f4 {
namespace tim {
namespace hall {

struct input_pin_config {
  gpio::port port;
  gpio::pin pin;
  gpio::pull pull;
};

struct hall_interface_config {
  clock_division clkdiv;
  std::optional<uint16_t> prescaler;
  std::optional<uint16_t> period;
  nvic::irq_priority irq_priority;
  std::array<input_pin_config, 3> pins;
};

namespace detail {

void configure_timebase(registers& regs, hall_interface_config const& conf);

void configure_channel(registers& regs);

template<timer_instance Tim>
[[nodiscard]] gpio::alternate_pin_config
make_input_config(input_pin_config const& pin) {
  return gpio::alternate_pin_config{
      .port = pin.port,
      .pin = pin.pin,
      .pull = pin.pull,
      .output_type = gpio::output_type::pushpull,
      .speed = gpio::speed::low,
      .altfunc = Tim::gpio_altfunc
  };
}

}

template<timer_instance Tim>
  requires(
      std::same_as<typename Tim::counter_type, uint32_t> &&
      Tim::io_channel_count >= 3
  )
class hall_interface {
public:
  using timer_instance = Tim;
  using reg_addr = timer_instance::reg_addr;
private:
  static inline registers& regs_ = timer_instance::regs;
  static inline nvic::irq_number const irqn_ =
      timer_instance::capture_compare_irqn;

  std::array<std::optional<gpio::alternate_pin>, 3> pins_;
public:
  hall_interface(hall_interface_config conf) {
    timer_instance::enable_clock();

    for (size_t i = 0; i < 3; ++i) {
      pins_[i].emplace(detail::make_input_config<timer_instance>(conf.pins[i]));
    }

    if (!conf.prescaler.has_value()) {
      conf.prescaler = timer_instance::template clock_frequency<uint32_t>() /
                       1'000'0000;
    }

    if (!conf.period.has_value()) {
      conf.period =
          std::numeric_limits<typename timer_instance::counter_type>::max();
    }

    detail::configure_timebase(regs_, conf);
    detail::configure_channel(regs_);

    // Interrupt configuration
    regs_.DIEN_B.CC1IEN = 1;
    regs_.DIEN_B.UIEN = 1;
    set_irq_priority(irqn_, conf.irq_priority);
  }

  registers& regs() {
    return regs_;
  }

  void enable() {
    acknowledge_update<timer_instance>();
    nvic::clear_pending_irq(irqn_);
    nvic::enable_irq(irqn_);
    enable_counter<timer_instance>();
  }

  typename timer_instance::counter_type capture_value() const {
    return emb::mmio::reg<reg_addr::ccrx[0]>::read();
  }

  std::array<emb::gpio::level, 3> input_levels() const {
    return {
        pins_[0]->read_level(),
        pins_[1]->read_level(),
        pins_[2]->read_level()
    };
  }

  uint8_t input_state() const {
    uint8_t state = std::to_underlying(pins_[0]->read_level()) |
                    std::to_underlying(pins_[1]->read_level()) << 1 |
                    std::to_underlying(pins_[2]->read_level()) << 2;
    return state;
  }
};

}
} // namespace tim
} // namespace f4
} // namespace apm32
