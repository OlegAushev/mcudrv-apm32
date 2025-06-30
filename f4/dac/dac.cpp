#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/dac/dac.hpp>

namespace mcu {
inline namespace apm32 {
namespace dac {

Module::Module(Peripheral peripheral)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{detail::regs[std::to_underlying(peripheral)]} {
  enable_clk(peripheral);
}

std::unique_ptr<gpio::AnalogPin> Module::initialize_channel(
    Channel channel, PinConfig const& pinconf, ChannelConfig const& conf) {
  auto pin{std::make_unique<gpio::AnalogPin>(
      gpio::AnalogConfig{.port = pinconf.port, .pin = pinconf.pin})};
  DAC_Config(std::to_underlying(channel),
             const_cast<DAC_Config_T*>(&conf.hal_config));
  DAC_Enable(static_cast<DAC_CHANNEL_T>(channel));

  return pin;
}

void Module::enable_clk(Peripheral peripheral) {
  auto dac_idx = std::to_underlying(peripheral);
  if (clk_enabled_[dac_idx]) {
    return;
  }

  detail::clk_enable_funcs[dac_idx]();
  clk_enabled_[dac_idx] = true;
}

} // namespace dac
} // namespace apm32
} // namespace mcu

#endif
#endif
