#pragma once

#include <apm32/f4/core/core.hpp>
#include <apm32/f4/gpio/output_pin.hpp>

#include <emb/mmio.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

namespace apm32::f4::dbg {

enum class probe_mode { level, edge };

enum class probe_channel : std::uint32_t {
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
  static inline std::uint32_t min_pulse_cycles_ = 0;
  gpio::output_pin* const pin_;
  probe_mode const mode_;
  std::uint32_t start_ = 0;
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
            .speed = gpio::speed::very_high,
            .polarity = emb::gpio::polarity::active_high
        }
    );
  }

  probe(probe_channel ch, probe_mode mode)
      : pin_(
            pins_[std::to_underlying(ch)].has_value()
                ? &pins_[std::to_underlying(ch)].value()
                : nullptr
        ),
        mode_(mode) {
    if (!pin_) {
      return;
    }

    if (mode_ == probe_mode::level) {
      pin_->set_level(emb::gpio::level::high);
      start_ = stamp();
    } else {
      pin_->toggle();
    }
  }

  ~probe() {
    if (!pin_) {
      return;
    }

    if (mode_ == probe_mode::level) {
      wait_min_pulse(start_);
      pin_->set_level(emb::gpio::level::low);
    } else {
      pulse();
    }
  }

  static void set_min_pulse(std::uint32_t cycles) {
    // wait_min_pulse() terminates only if a nonzero minimum
    // never exists without a running CYCCNT
    emb::mmio::set<CoreDebug_DEMCR_TRCENA_Msk>(CoreDebug->DEMCR);
    emb::mmio::set<DWT_CTRL_CYCCNTENA_Msk>(DWT->CTRL);
    min_pulse_cycles_ = cycles;
  }

  void mark() {
    if (!pin_) {
      return;
    }

    pulse();
  }

  static std::optional<emb::gpio::level> read(probe_channel ch) {
    auto const pin = pins_[std::to_underlying(ch)].has_value()
                       ? &pins_[std::to_underlying(ch)].value()
                       : nullptr;

    if (!pin) {
      return std::nullopt;
    }

    return pin->read_level();
  }
private:
  void pulse() {
    pin_->toggle();
    wait_min_pulse(stamp());
    pin_->toggle();
  }

  // DWT register reads are unpredictable while TRCENA is cleared,
  // so CYCCNT must stay untouched until set_min_pulse() enables it
  static std::uint32_t stamp() {
    return min_pulse_cycles_ != 0 ? DWT->CYCCNT : 0;
  }

  // unsigned wraparound arithmetic: correct across CYCCNT overflow
  static void wait_min_pulse(std::uint32_t start) {
    if (min_pulse_cycles_ == 0) {
      return;
    }

    while (DWT->CYCCNT - start < min_pulse_cycles_) {}
  }
};

} // namespace apm32::f4::dbg
