#include <apm32/f4/dac.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace dac {

peripheral::peripheral(peripheral_id id)
    : peripheral_type(id),
      id_(id),
      regs_(peripherals[std::to_underlying(id)]) {
  enable_clock(id_);
}

std::unique_ptr<gpio::analog_pin> peripheral::configure_channel(
    channel ch,
    pin_config const& pinconf,
    channel_config chconf
) {
  auto pin = std::make_unique<gpio::analog_pin>(
      gpio::analog_pin_config{.port = pinconf.port, .pin = pinconf.pin}
  );

  uint32_t const offset = std::to_underlying(ch);

  // configure channel: buffer disable, trigger selection, trigger enable
  uint32_t ctrl_val = 0;
  if (chconf.output_buffer_disable) {
    ctrl_val |= (1u << 1);    // BUFFDCHx
  }
  if (chconf.trigger_enable) {
    ctrl_val |= (1u << 2);    // TRGENCHx
    ctrl_val |= (chconf.trigger_selection & 0x7u) << 3;  // TRGSELCHx
  }

  // clear the 14 control bits for this channel and set new values
  uint32_t const ch_mask = 0x3FFFu << offset;
  emb::mmio::write(regs_->CTRL, ch_mask, ctrl_val);

  // enable channel
  emb::mmio::set(regs_->CTRL, 1u << offset);  // ENCHx

  return pin;
}

void peripheral::enable_clock(peripheral_id id) {
  auto dac_idx = std::to_underlying(id);
  if (is_clock_enabled_[dac_idx]) {
    return;
  }

  emb::mmio::set(RCM->APB1CLKEN, detail::clock_bits[dac_idx]);
  is_clock_enabled_[dac_idx] = true;
}

} // namespace dac
} // namespace f4
} // namespace apm32
