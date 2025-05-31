#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/i2c/i2c.hpp>

namespace mcu {
namespace apm32 {
namespace i2c {

Module::Module(Peripheral peripheral,
               SdaPinConfig const& sda_pin_conf,
               SclPinConfig const& scl_pin_conf,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_(detail::regs[std::to_underlying(peripheral)]),
      sda_pin_{std::make_unique<gpio::AlternatePin>(
          gpio::AlternatePinConfig{.port = sda_pin_conf.port,
                                   .pin = sda_pin_conf.pin,
                                   .pull = gpio::Pull::none,
                                   .output = gpio::Output::opendrain,
                                   .speed = gpio::Speed::fast,
                                   .altfunc = sda_pin_conf.altfunc})},
      scl_pin_{std::make_unique<gpio::AlternatePin>(
          gpio::AlternatePinConfig{.port = scl_pin_conf.port,
                                   .pin = scl_pin_conf.pin,
                                   .pull = gpio::Pull::none,
                                   .output = gpio::Output::opendrain,
                                   .speed = gpio::Speed::fast,
                                   .altfunc = scl_pin_conf.altfunc})},
      conf_{conf} {
  enable_clk(peripheral);
  I2C_Config(regs_, const_cast<I2C_Config_T*>(&conf.hal_config));
}

void Module::enable_clk(Peripheral peripheral) {
  size_t i2c_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[i2c_idx]) {
    return;
  }

  detail::clk_enable_funcs[i2c_idx]();
  clk_enabled_[i2c_idx] = true;
}

} // namespace i2c
} // namespace apm32
} // namespace mcu

#endif
#endif
