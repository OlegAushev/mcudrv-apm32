#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/i2c/i2c.hpp>

namespace mcu {
namespace apm32 {
namespace i2c {

internal::SdaPin::SdaPin(SdaPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::opendrain,
                          .speed = gpio::Speed::fast,
                          .altfunc = conf.altfunc}} {}

internal::SclPin::SclPin(SclPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::opendrain,
                          .speed = gpio::Speed::fast,
                          .altfunc = conf.altfunc}} {}

Module::Module(Peripheral peripheral,
               SdaPinConfig const& sda_pinconf,
               SclPinConfig const& scl_pinconf,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_(i2c::regs[std::to_underlying(peripheral)]),
      sda_pin_{sda_pinconf},
      scl_pin_{scl_pinconf},
      conf_{conf} {
  enable_clk(peripheral);
  I2C_Config(regs_, const_cast<I2C_Config_T*>(&conf.hal_config));
}

void Module::enable_clk(Peripheral peripheral) {
  size_t i2c_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[i2c_idx]) {
    return;
  }

  enable_clk_[i2c_idx]();
  clk_enabled_[i2c_idx] = true;
}

} // namespace i2c
} // namespace apm32
} // namespace mcu

#endif
#endif
