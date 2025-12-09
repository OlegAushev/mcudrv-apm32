#include <apm32/f4/dac.hpp>

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
  DAC_Config(std::to_underlying(ch), &chconf.hal_config);
  DAC_Enable(static_cast<DAC_CHANNEL_T>(ch));

  return pin;
}

void peripheral::enable_clock(peripheral_id id) {
  auto dac_idx = std::to_underlying(id);
  if (is_clock_enabled_[dac_idx]) {
    return;
  }

  detail::enable_clock[dac_idx]();
  is_clock_enabled_[dac_idx] = true;
}

} // namespace dac
} // namespace f4
} // namespace apm32
