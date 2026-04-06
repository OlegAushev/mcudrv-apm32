#pragma once

#include <apm32/f4/adc/adc_instances.hpp>
#include <apm32/f4/adc/adc_types.hpp>

#include <apm32/f4/gpio.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace adc {

enum class channel_type {
  external,
  internal_temp,
  internal_vref
};

namespace detail {

// sample time: 3 bits per channel
// channels 0-9  -> SMPTIM2 at (ch * 3) bits
// channels 10-17 -> SMPTIM1 at ((ch - 10) * 3) bits
inline void set_sample_time(registers& regs, unsigned ch, uint8_t smp) {
  if (ch < 10) {
    uint32_t const pos = ch * 3u;
    emb::mmio::write(regs.SMPTIM2, 0x7u << pos, uint32_t(smp));
  } else {
    uint32_t const pos = (ch - 10u) * 3u;
    emb::mmio::write(regs.SMPTIM1, 0x7u << pos, uint32_t(smp));
  }
}

// regular sequence: 5 bits per rank
// ranks 1-6   -> REGSEQ3 at ((rank-1) * 5) bits
// ranks 7-12  -> REGSEQ2 at ((rank-7) * 5) bits
// ranks 13-16 -> REGSEQ1 at ((rank-13) * 5) bits
inline void set_regular_sequence(registers& regs, unsigned ch, uint8_t rank) {
  if (rank <= 6) {
    uint32_t const pos = (rank - 1u) * 5u;
    emb::mmio::write(regs.REGSEQ3, 0x1Fu << pos, uint32_t(ch));
  } else if (rank <= 12) {
    uint32_t const pos = (rank - 7u) * 5u;
    emb::mmio::write(regs.REGSEQ2, 0x1Fu << pos, uint32_t(ch));
  } else {
    uint32_t const pos = (rank - 13u) * 5u;
    emb::mmio::write(regs.REGSEQ1, 0x1Fu << pos, uint32_t(ch));
  }
}

// injected sequence: 5 bits per rank, ranks 1-4 at ((rank-1) * 5) bits
inline void set_injected_sequence(registers& regs, unsigned ch, uint8_t rank) {
  uint32_t const pos = (rank - 1u) * 5u;
  emb::mmio::write(regs.INJSEQ, 0x1Fu << pos, uint32_t(ch));
}

// injected offset: INJDOF1-4 registers
inline void set_injected_offset(registers& regs, uint8_t rank, uint16_t offset) {
  switch (rank) {
  case 1:
    regs.INJDOF1 = offset;
    break;
  case 2:
    regs.INJDOF2 = offset;
    break;
  case 3:
    regs.INJDOF3 = offset;
    break;
  case 4:
    regs.INJDOF4 = offset;
    break;
  }
}

inline void enable_temp_sensor_vrefint() {
  emb::mmio::set(ADC123_COMMON->CCTRL, ADC_CCTRL_TSVREFEN);
}

} // namespace detail

struct adc123_in0 {
  static constexpr unsigned idx = 0;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin0;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc123_in1 {
  static constexpr unsigned idx = 1;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin1;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc123_in2 {
  static constexpr unsigned idx = 2;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin2;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc123_in3 {
  static constexpr unsigned idx = 3;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin3;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc12_in4 {
  static constexpr unsigned idx = 4;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin4;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc12_in5 {
  static constexpr unsigned idx = 5;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin5;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc12_in6 {
  static constexpr unsigned idx = 6;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin6;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc12_in7 {
  static constexpr unsigned idx = 7;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioa;
  static constexpr auto pin = gpio::pin::pin7;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc12_in8 {
  static constexpr unsigned idx = 8;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiob;
  static constexpr auto pin = gpio::pin::pin0;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc12_in9 {
  static constexpr unsigned idx = 9;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiob;
  static constexpr auto pin = gpio::pin::pin1;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc123_in10 {
  static constexpr unsigned idx = 10;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioc;
  static constexpr auto pin = gpio::pin::pin0;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc123_in11 {
  static constexpr unsigned idx = 11;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioc;
  static constexpr auto pin = gpio::pin::pin1;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc123_in12 {
  static constexpr unsigned idx = 12;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioc;
  static constexpr auto pin = gpio::pin::pin2;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc123_in13 {
  static constexpr unsigned idx = 13;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioc;
  static constexpr auto pin = gpio::pin::pin3;
  using adc_instances = emb::typelist<adc1, adc2, adc3>;
};

struct adc12_in14 {
  static constexpr unsigned idx = 14;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioc;
  static constexpr auto pin = gpio::pin::pin4;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc12_in15 {
  static constexpr unsigned idx = 15;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpioc;
  static constexpr auto pin = gpio::pin::pin5;
  using adc_instances = emb::typelist<adc1, adc2>;
};

struct adc3_in4 {
  static constexpr unsigned idx = 4;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin6;
  using adc_instances = emb::typelist<adc3>;
};

struct adc3_in5 {
  static constexpr unsigned idx = 5;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin7;
  using adc_instances = emb::typelist<adc3>;
};

struct adc3_in6 {
  static constexpr unsigned idx = 6;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin8;
  using adc_instances = emb::typelist<adc3>;
};

struct adc3_in7 {
  static constexpr unsigned idx = 7;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin9;
  using adc_instances = emb::typelist<adc3>;
};

struct adc3_in8 {
  static constexpr unsigned idx = 8;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin10;
  using adc_instances = emb::typelist<adc3>;
};

struct adc3_in9 {
  static constexpr unsigned idx = 9;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin3;
  using adc_instances = emb::typelist<adc3>;
};

struct adc3_in14 {
  static constexpr unsigned idx = 14;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin4;
  using adc_instances = emb::typelist<adc3>;
};

struct adc3_in15 {
  static constexpr unsigned idx = 15;
  static constexpr auto type = channel_type::external;
  static constexpr auto port = gpio::port::gpiof;
  static constexpr auto pin = gpio::pin::pin5;
  using adc_instances = emb::typelist<adc3>;
};

struct adc1_in16 {
  static constexpr unsigned idx = 16;
  static constexpr auto type = channel_type::internal_temp;
  using adc_instances = emb::typelist<adc1>;
};

struct adc1_in17 {
  static constexpr unsigned idx = 17;
  static constexpr auto type = channel_type::internal_vref;
  using adc_instances = emb::typelist<adc1>;
};

template<typename Channel, sampletime Sampletime, unsigned... Ranks>
  requires((1 <= Ranks && Ranks <= 4) && ...)
struct injected_channel {
  using channel = Channel;
  static constexpr auto sampletime = Sampletime;
  static constexpr std::array ranks = {Ranks...};

  static std::optional<gpio::analog_pin_config> init(registers& regs) {
    for (auto rank : ranks) {
      detail::set_sample_time(regs, channel::idx, static_cast<uint8_t>(sampletime));
      detail::set_injected_sequence(regs, channel::idx, static_cast<uint8_t>(rank));
      detail::set_injected_offset(regs, static_cast<uint8_t>(rank), 0);
    }

    if constexpr (channel::type == channel_type::external) {
      return gpio::analog_pin_config{
          .port = channel::port,
          .pin = channel::pin
      };
    } else {
      detail::enable_temp_sensor_vrefint();
      return std::nullopt;
    }
  }
};

template<typename Channel, sampletime Sampletime, unsigned... Ranks>
  requires((1 <= Ranks && Ranks <= 16) && ...)
struct regular_channel {
  using channel = Channel;
  static constexpr auto sampletime = Sampletime;
  static constexpr std::array ranks = {Ranks...};

  static std::optional<gpio::analog_pin_config> init(registers& regs) {
    for (auto rank : ranks) {
      detail::set_sample_time(regs, channel::idx, static_cast<uint8_t>(sampletime));
      detail::set_regular_sequence(regs, channel::idx, static_cast<uint8_t>(rank));
    }

    if constexpr (channel::type == channel_type::external) {
      return gpio::analog_pin_config{
          .port = channel::port,
          .pin = channel::pin
      };
    } else {
      detail::enable_temp_sensor_vrefint();
      return std::nullopt;
    }
  }
};

} // namespace adc
} // namespace f4
} // namespace apm32
