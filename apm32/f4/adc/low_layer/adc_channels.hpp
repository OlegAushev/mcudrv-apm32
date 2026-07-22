#pragma once

#include <apm32/f4/adc/low_layer/adc_instances.hpp>
#include <apm32/f4/adc/low_layer/adc_types.hpp>

#include <apm32/f4/gpio/gpio.hpp>

#include <emb/mmio.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace apm32::f4::adc {

namespace detail {

template<std::size_t N>
constexpr bool
ranks_in_range(std::array<unsigned, N> const& ranks, unsigned lo, unsigned hi) {
  for (auto rank : ranks) {
    if (rank < lo || rank > hi) {
      return false;
    }
  }
  return true;
}

template<std::size_t N>
constexpr bool ranks_unique(std::array<unsigned, N> const& ranks) {
  for (std::size_t i = 0; i < N; ++i) {
    for (std::size_t j = i + 1; j < N; ++j) {
      if (ranks[i] == ranks[j]) {
        return false;
      }
    }
  }
  return true;
}

} // namespace detail

template<unsigned... Ranks>
struct injected_rank_sequence {
  static constexpr std::array<unsigned, sizeof...(Ranks)> values{Ranks...};

  static_assert(
      sizeof...(Ranks) >= 1,
      "injected_rank_sequence must contain at least one rank"
  );
  static_assert(
      detail::ranks_in_range(values, 1u, 4u),
      "injected ranks must be in [1, 4]"
  );
  static_assert(
      detail::ranks_unique(values),
      "injected ranks must be unique (no slot may be assigned twice)"
  );
};

template<unsigned... Ranks>
struct regular_rank_sequence {
  static constexpr std::array<unsigned, sizeof...(Ranks)> values{Ranks...};

  static_assert(
      sizeof...(Ranks) >= 1,
      "regular_rank_sequence must contain at least one rank"
  );
  static_assert(
      detail::ranks_in_range(values, 1u, 16u),
      "regular ranks must be in [1, 16]"
  );
  static_assert(
      detail::ranks_unique(values),
      "regular ranks must be unique (no slot may be assigned twice)"
  );
};

template<typename T>
inline constexpr bool is_injected_sequence = false;

template<unsigned... R>
inline constexpr bool is_injected_sequence<injected_rank_sequence<R...>> = true;

template<typename T>
inline constexpr bool is_regular_sequence = false;

template<unsigned... R>
inline constexpr bool is_regular_sequence<regular_rank_sequence<R...>> = true;

enum class channel_type { external, internal_temp, internal_vref };

namespace detail {

// sample time: 3 bits per channel
// channels 0-9  -> SMPTIM2 at (ch * 3) bits
// channels 10-17 -> SMPTIM1 at ((ch - 10) * 3) bits
inline void set_sample_time(registers& reg, unsigned ch, std::uint8_t smp) {
  if (ch < 10) {
    std::uint32_t const pos = ch * 3u;
    emb::mmio::runtime::write(reg.SMPTIM2, 0x7u << pos, std::uint32_t(smp));
  } else {
    std::uint32_t const pos = (ch - 10u) * 3u;
    emb::mmio::runtime::write(reg.SMPTIM1, 0x7u << pos, std::uint32_t(smp));
  }
}

// regular sequence: 5 bits per rank
// ranks 1-6   -> REGSEQ3 at ((rank-1) * 5) bits
// ranks 7-12  -> REGSEQ2 at ((rank-7) * 5) bits
// ranks 13-16 -> REGSEQ1 at ((rank-13) * 5) bits
inline void
set_regular_sequence(registers& reg, unsigned ch, std::uint8_t rank) {
  if (rank <= 6) {
    std::uint32_t const pos = (rank - 1u) * 5u;
    emb::mmio::runtime::write(reg.REGSEQ3, 0x1Fu << pos, std::uint32_t(ch));
  } else if (rank <= 12) {
    std::uint32_t const pos = (rank - 7u) * 5u;
    emb::mmio::runtime::write(reg.REGSEQ2, 0x1Fu << pos, std::uint32_t(ch));
  } else {
    std::uint32_t const pos = (rank - 13u) * 5u;
    emb::mmio::runtime::write(reg.REGSEQ1, 0x1Fu << pos, std::uint32_t(ch));
  }
}

// injected sequence: 5 bits per slot, slots 1-4 at ((slot-1) * 5) bits
// (right-alignment by sequence length is handled by the caller)
inline void
set_injected_sequence(registers& reg, unsigned ch, std::uint8_t slot) {
  std::uint32_t const pos = (slot - 1u) * 5u;
  emb::mmio::runtime::write(reg.INJSEQ, 0x1Fu << pos, std::uint32_t(ch));
}

// injected offset: INJDOF1-4 registers
inline void
set_injected_offset(registers& reg, std::uint8_t rank, std::uint16_t offset) {
  switch (rank) {
  case 1: reg.INJDOF1 = offset; break;
  case 2: reg.INJDOF2 = offset; break;
  case 3: reg.INJDOF3 = offset; break;
  case 4: reg.INJDOF4 = offset; break;
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

template<typename Channel, sampletime Sampletime, typename Ranks>
  requires(is_injected_sequence<Ranks> || is_regular_sequence<Ranks>)
struct channel {
  using descriptor = Channel;
  static constexpr bool injected = is_injected_sequence<Ranks>;
  static constexpr auto sampletime = Sampletime;
  static constexpr std::array ranks = Ranks::values;

  static constexpr unsigned rank()
    requires(Ranks::values.size() == 1) {
    return ranks[0];
  }

  static std::optional<gpio::analog_pin_config> init(registers& reg)
    requires is_regular_sequence<Ranks> {
    using namespace detail;
    set_sample_time(reg, Channel::idx, static_cast<std::uint8_t>(sampletime));
    for (auto rank : ranks) {
      set_regular_sequence(reg, Channel::idx, static_cast<std::uint8_t>(rank));
    }
    return finalize();
  }

  static std::optional<gpio::analog_pin_config>
  init(registers& reg, unsigned injected_count)
    requires is_injected_sequence<Ranks> {
    using namespace detail;
    set_sample_time(reg, Channel::idx, static_cast<std::uint8_t>(sampletime));
    for (auto rank : ranks) {
      // INJSEQ is right-aligned: for N conversions the sequence occupies
      // slots INJSEQC(4-N+1)..INJSEQC4, so conversion order `rank` (1..N)
      // maps to slot (4 - N + rank). Result lands in INJDATA[rank].
      auto const slot = static_cast<std::uint8_t>(4u - injected_count + rank);
      set_injected_sequence(reg, Channel::idx, slot);
      set_injected_offset(reg, static_cast<std::uint8_t>(rank), 0);
    }
    return finalize();
  }

private:
  static std::optional<gpio::analog_pin_config> finalize() {
    if constexpr (Channel::type == channel_type::external) {
      return gpio::analog_pin_config{
          .port = Channel::port,
          .pin = Channel::pin
      };
    } else {
      detail::enable_temp_sensor_vrefint();
      return std::nullopt;
    }
  }
};

template<typename T>
inline constexpr bool is_adc_channel = false;

template<typename Channel, sampletime Sampletime, typename Ranks>
inline constexpr bool is_adc_channel<channel<Channel, Sampletime, Ranks>> =
    true;

template<typename T>
concept some_adc_channel = is_adc_channel<T>;

} // namespace apm32::f4::adc
