#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include "pwm.hpp"

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace tim {
namespace adv {

PwmTimer::PwmTimer(Peripheral peripheral,
                   PwmConfig const& conf,
                   BkinPinConfig const* bkin_conf)
    : internal::AbstractTimer(peripheral, OpMode::pwm_generation) {
  auto cfg = conf;

  float timebase_freq =
      float(core_clk_freq()) / float(conf.hal_base_config.division + 1);

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

  switch (conf.hal_base_config.clockDivision) {
  case TMR_CLOCK_DIV_1:
    t_dts_ns_ = float(conf.hal_base_config.division + 1) * 1000000000.f /
                float(core_clk_freq());
    break;
  case TMR_CLOCK_DIV_2:
    t_dts_ns_ = 2 * float(conf.hal_base_config.division + 1) * 1000000000.f /
                float(core_clk_freq());
    break;
  case TMR_CLOCK_DIV_4:
    t_dts_ns_ = 4 * float(conf.hal_base_config.division + 1) * 1000000000.f /
                float(core_clk_freq());
    break;
  }

  if (conf.arr_preload) {
    regs_->CTRL1_B.ARPEN = 1;
  }

  TMR_ConfigTimeBase(regs_, &cfg.hal_base_config);

  init_bdt(conf, bkin_conf);
}

void PwmTimer::init_bdt(PwmConfig const& conf, BkinPinConfig const* bkin_conf) {
  if (conf.hal_bdt_config.BRKState == TMR_BRK_STATE_ENABLE &&
      bkin_conf == nullptr) {
    fatal_error();
  }

  if ((conf.hal_bdt_config.BRKPolarity == TMR_BRK_POLARITY_LOW &&
       bkin_conf->active_level != emb::gpio::level::low) ||
      (conf.hal_bdt_config.BRKPolarity == TMR_BRK_POLARITY_HIGH &&
       bkin_conf->active_level != emb::gpio::level::high)) {
    fatal_error();
  }

  if (bkin_conf != nullptr) {
    bkin_pin_ =
        std::make_unique<mcu::apm32::tim::internal::BkinPin>(*bkin_conf);
  }

  auto bdt_conf{conf.hal_bdt_config};

  if (bdt_conf.deadTime == 0) {
    // deadtime specified by deadtime_ns
    deadtime_ = conf.deadtime_ns * 1E-09f;
    if (conf.deadtime_ns <= 0X7F * t_dts_ns_) {
      bdt_conf.deadTime = uint16_t(conf.deadtime_ns / t_dts_ns_);
    } else if (conf.deadtime_ns <= 127 * 2 * t_dts_ns_) {
      bdt_conf.deadTime =
          uint16_t((conf.deadtime_ns - 64 * 2 * t_dts_ns_) / (2 * t_dts_ns_));
      bdt_conf.deadTime |= 0x80;
    } else if (conf.deadtime_ns <= 63 * 8 * t_dts_ns_) {
      bdt_conf.deadTime =
          uint16_t((conf.deadtime_ns - 32 * 8 * t_dts_ns_) / (8 * t_dts_ns_));
      bdt_conf.deadTime |= 0xC0;
    } else if (conf.deadtime_ns <= 63 * 16 * t_dts_ns_) {
      bdt_conf.deadTime =
          uint16_t((conf.deadtime_ns - 32 * 16 * t_dts_ns_) / (16 * t_dts_ns_));
      bdt_conf.deadTime |= 0xE0;
    } else {
      fatal_error();
    }
  } else {
    auto dtg{bdt_conf.deadTime};
    if ((dtg & 0x80) == 0) {
      deadtime_ = float(dtg) * t_dts_ns_ * 1E-09f;
    } else if ((dtg & 0xC0) == 0x80) {
      deadtime_ = float(64 + (dtg & 0x3F)) * 2 * t_dts_ns_ * 1E-09f;
    } else if ((dtg & 0xE0) == 0xC0) {
      deadtime_ = float(32 + (dtg & 0x1F)) * 8 * t_dts_ns_ * 1E-09f;
    } else if ((dtg & 0xE0) == 0xE0) {
      deadtime_ = float(32 + (dtg & 0x1F)) * 16 * t_dts_ns_ * 1E-09f;
    } else {
      fatal_error();
    }
  }

  TMR_ConfigBDT(regs_, &bdt_conf);
}

void PwmTimer::init_channel(Channel channel,
                            ChPinConfig const* ch_conf,
                            ChPinConfig const* chn_conf,
                            PwmChannelConfig const& conf) {
  if (ch_conf) {
    ch_pins_[std::to_underlying(channel)].first =
        std::make_unique<mcu::apm32::tim::internal::ChPin>(*ch_conf);
  }

  if (chn_conf) {
    ch_pins_[std::to_underlying(channel)].second =
        std::make_unique<mcu::apm32::tim::internal::ChPin>(*chn_conf);
  }

  switch (channel) {
  case Channel::channel1:
    regs_->CCM1_COMPARE_B.OC1PEN = conf.oc_preload & 0x01;
    TMR_ConfigOC1(regs_, const_cast<TMR_OCConfig_T*>(&conf.hal_oc_config));
    if (ch_conf) {
      regs_->CCEN_B.CC1EN = 1;
    } else {
      regs_->CCEN_B.CC1EN = 0;
    }
    if (chn_conf) {
      regs_->CCEN_B.CC1NEN = 1;
    } else {
      regs_->CCEN_B.CC1NEN = 0;
    }
    break;
  case Channel::channel2:
    regs_->CCM1_COMPARE_B.OC2PEN = conf.oc_preload & 0x01;
    TMR_ConfigOC2(regs_, const_cast<TMR_OCConfig_T*>(&conf.hal_oc_config));
    if (ch_conf) {
      regs_->CCEN_B.CC2EN = 1;
    } else {
      regs_->CCEN_B.CC2EN = 0;
    }
    if (chn_conf) {
      regs_->CCEN_B.CC2NEN = 1;
    } else {
      regs_->CCEN_B.CC2NEN = 0;
    }
    break;
  case Channel::channel3:
    regs_->CCM2_COMPARE_B.OC3PEN = conf.oc_preload & 0x01;
    TMR_ConfigOC3(regs_, const_cast<TMR_OCConfig_T*>(&conf.hal_oc_config));
    if (ch_conf) {
      regs_->CCEN_B.CC3EN = 1;
    } else {
      regs_->CCEN_B.CC3EN = 0;
    }
    if (chn_conf) {
      regs_->CCEN_B.CC3NEN = 1;
    } else {
      regs_->CCEN_B.CC3NEN = 0;
    }
    break;
  case Channel::channel4:
    regs_->CCM2_COMPARE_B.OC4PEN = conf.oc_preload & 0x01;
    TMR_ConfigOC4(regs_, const_cast<TMR_OCConfig_T*>(&conf.hal_oc_config));
    if (ch_conf) {
      regs_->CCEN_B.CC4EN = 1;
    } else {
      regs_->CCEN_B.CC4EN = 0;
    }
    break;
  }
}

void PwmTimer::init_update_interrupts(IrqPriority priority) {
  regs_->DIEN_B.UIEN = 1;
  set_irq_priority(internal::up_irq_nums[std::to_underlying(peripheral_)],
                   priority);
}

void PwmTimer::init_break_interrupts(IrqPriority priority) {
  regs_->DIEN_B.BRKIEN = 1;
  set_irq_priority(internal::brk_irq_nums[std::to_underlying(peripheral_)],
                   priority);
  brk_enabled_ = true;
}

} // namespace adv
} // namespace tim
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
#endif
