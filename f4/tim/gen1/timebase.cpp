#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/tim/gen1/timebase.hpp>

namespace mcu {
namespace apm32 {
namespace tim {
namespace gen1 {

TimebaseTimer::TimebaseTimer(Peripheral peripheral, TimebaseConfig const& conf)
    : internal::AbstractTimer(peripheral, OpMode::timebase) {
  auto cfg{conf};

  float timebase_freq =
      (float(core_clk_freq()) / 2) / float(conf.hal_base_config.division + 1);

  if (cfg.hal_base_config.period == 0 && cfg.freq != 0) {
    // period specified by freq
    freq_ = conf.freq;
    switch (conf.hal_base_config.countMode) {
    case TMR_COUNTER_MODE_UP:
    case TMR_COUNTER_MODE_DOWN:
      cfg.hal_base_config.period = uint32_t((timebase_freq / conf.freq) - 1);
      break;
    case TMR_COUNTER_MODE_CENTER_ALIGNED1:
    case TMR_COUNTER_MODE_CENTER_ALIGNED2:
    case TMR_COUNTER_MODE_CENTER_ALIGNED3:
      cfg.hal_base_config.period = uint32_t((timebase_freq / conf.freq) / 2);
      break;
    }
  } else if (conf.hal_base_config.period != 0) {
    // period specified by hal_base_config.period
    switch (conf.hal_base_config.countMode) {
    case TMR_COUNTER_MODE_UP:
    case TMR_COUNTER_MODE_DOWN:
      freq_ = timebase_freq / float(cfg.hal_base_config.period + 1);
      break;
    case TMR_COUNTER_MODE_CENTER_ALIGNED1:
    case TMR_COUNTER_MODE_CENTER_ALIGNED2:
    case TMR_COUNTER_MODE_CENTER_ALIGNED3:
      freq_ = timebase_freq / float(cfg.hal_base_config.period * 2);
      break;
    }
  } else {
    fatal_error();
  }

  if (cfg.hal_base_config.period > UINT16_MAX) {
    fatal_error();
  }

  if (conf.arr_preload) {
    regs_->CTRL1_B.ARPEN = 1;
  }

  TMR_ConfigTimeBase(regs_, &cfg.hal_base_config);
}

} // namespace gen1
} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
