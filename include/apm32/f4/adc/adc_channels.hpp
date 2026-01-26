#pragma once

#include <apm32/f4/adc/adc_instances.hpp>
#include <apm32/f4/adc/adc_types.hpp>

#include <apm32/f4/gpio.hpp>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

enum class channel_type {
  external,
  internal_temp,
  internal_vref
};

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
      ADC_ConfigInjectedChannel(
          &regs,
          channel::idx,
          static_cast<uint8_t>(rank),
          static_cast<uint8_t>(sampletime)
      );
      ADC_ConfigInjectedOffset(
          &regs,
          static_cast<ADC_INJEC_CHANNEL_T>(rank),
          0
      );
    }

    if constexpr (channel::type == channel_type::external) {
      return gpio::analog_pin_config{
          .port = channel::port,
          .pin = channel::pin
      };
    } else {
      ADC_EnableTempSensorVrefint();
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
      ADC_ConfigRegularChannel(
          &regs,
          channel::idx,
          static_cast<uint8_t>(rank),
          static_cast<uint8_t>(sampletime)
      );
    }

    if constexpr (channel::type == channel_type::external) {
      return gpio::analog_pin_config{
          .port = channel::port,
          .pin = channel::pin
      };
    } else {
      ADC_EnableTempSensorVrefint();
      return std::nullopt;
    }
  }
};

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
