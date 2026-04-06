#include <apm32/f4/gpio.hpp>

#include <apm32/f4/core.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace gpio {

detail::pin_base::pin_base(
    port p,
    uint16_t pin_mask,
    uint32_t mode_val,
    uint32_t otype_val,
    uint32_t speed_val,
    uint32_t pupd_val,
    std::optional<uint32_t> altfunc
)
    : port_(p), pin_(pin_mask), regs_(ports[std::to_underlying(p)]) {
  size_t const port_idx = std::to_underlying(port_);
  core::ensure(!(used_pins_[port_idx] & pin_));
  used_pins_[port_idx] |= uint16_t(pin_);

  if (!is_clock_enabled_[port_idx]) {
    emb::mmio::set(RCM->AHB1CLKEN, port_clock_bits_[port_idx]);
    is_clock_enabled_[port_idx] = true;
  }

  auto const pin_no = bit_position(pin_);
  auto const pin2 = pin_no * 2u;

  // configure alternate function (before setting mode)
  if (mode_val == mode::alternate && altfunc.has_value()) {
    uint32_t const alf_idx = pin_no / 8u;
    uint32_t const alf_pos = (pin_no % 8u) * 4u;
    emb::mmio::write(regs_->ALF[alf_idx], 0xFu << alf_pos, altfunc.value());
  }

  // MODE register (2 bits per pin)
  emb::mmio::write(regs_->MODE, 0x3u << pin2, mode_val);

  // OMODE register (1 bit per pin)
  emb::mmio::write(regs_->OMODE, 0x1u << pin_no, otype_val);

  // OSSEL register (2 bits per pin)
  emb::mmio::write(regs_->OSSEL, 0x3u << pin2, speed_val);

  // PUPD register (2 bits per pin)
  emb::mmio::write(regs_->PUPD, 0x3u << pin2, pupd_val);
}

detail::pin_base::~pin_base() {
  size_t const port_idx = std::to_underlying(port_);
  used_pins_[port_idx] &= ~uint16_t(pin_);
}

detail::pin_base::pin_base(input_pin_config const& conf)
    : pin_base(
          conf.port,
          std::to_underlying(conf.pin),
          mode::input,
          std::to_underlying(output_type::pushpull),
          std::to_underlying(speed::low),
          std::to_underlying(conf.pull)
      ) {}

detail::pin_base::pin_base(output_pin_config const& conf)
    : pin_base(
          conf.port,
          std::to_underlying(conf.pin),
          mode::output,
          std::to_underlying(conf.output_type),
          std::to_underlying(conf.speed),
          std::to_underlying(conf.pull)
      ) {}

detail::pin_base::pin_base(alternate_pin_config const& conf)
    : pin_base(
          conf.port,
          std::to_underlying(conf.pin),
          mode::alternate,
          std::to_underlying(conf.output_type),
          std::to_underlying(conf.speed),
          std::to_underlying(conf.pull),
          conf.altfunc
      ) {}

detail::pin_base::pin_base(analog_pin_config const& conf)
    : pin_base(
          conf.port,
          std::to_underlying(conf.pin),
          mode::analog,
          std::to_underlying(output_type::pushpull),
          std::to_underlying(speed::low),
          std::to_underlying(pull::none)
      ) {}

} // namespace gpio
} // namespace f4
} // namespace apm32
