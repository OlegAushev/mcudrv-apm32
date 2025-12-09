#pragma once

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>

namespace apm32 {
namespace f4 {
namespace dbg {

enum class probe_mode { level, edge };

enum class probe_channel : uint32_t {
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

class probe {
private:
  static inline std::array<std::optional<gpio::output_pin>, 16> pins_{};
  gpio::output_pin* const pin_;
  probe_mode const mode_;
public:
  probe(probe const&) = delete;
  probe(probe&&) = delete;
  probe& operator=(probe const&) = delete;
  probe& operator=(probe&&) = delete;

  static void configure(probe_channel ch, gpio::port prt, gpio::pin pn) {
    pins_[std::to_underlying(ch)].emplace(
        gpio::output_pin_config{
            .port = prt,
            .pin = pn,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = gpio::speed::high,
            .active_level = emb::gpio::level::high
        }
    );
  }

  probe(probe_channel ch, probe_mode mode)
      : pin_(
            pins_[std::to_underlying(ch)].has_value() ?
                &pins_[std::to_underlying(ch)].value() :
                nullptr
        ),
        mode_(mode) {
    if (!pin_) {
      return;
    }

    if (mode_ == probe_mode::level) {
      pin_->set_level(emb::gpio::level::high);
    } else {
      pin_->toggle();
    }
  }

  ~probe() {
    if (!pin_) {
      return;
    }

    if (mode_ == probe_mode::level) {
      pin_->set_level(emb::gpio::level::low);
    } else {
      pin_->toggle();
      pulse_delay();
      pin_->toggle();
    }
  }

  void mark() {
    if (!pin_) {
      return;
    }

    pin_->toggle();
    pulse_delay();
    pin_->toggle();
  }

  static std::optional<emb::gpio::level> read(probe_channel ch) {
    auto const pin = pins_[std::to_underlying(ch)].has_value() ?
                         &pins_[std::to_underlying(ch)].value() :
                         nullptr;

    if (!pin) {
      return std::nullopt;
    }

    return pin->read_level();
  }
private:
  void pulse_delay() {
    __NOP();
  }
};

} // namespace dbg
} // namespace f4
} // namespace apm32
