#pragma once

#include <apm32/f4/adc/adc.hpp>
#include <apm32/f4/adc/adc_sequence.hpp>
#include <apm32/f4/adc/common_adc.hpp>
#include <apm32/f4/dma/pm_stream.hpp>
#include <apm32/f4/gpio/analog_pin.hpp>

#include <emb/meta/typelist.hpp>
#include <emb/meta/unroll.hpp>
#include <emb/mmio.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace apm32::f4::adc {

namespace detail {

template<typename T>
consteval bool is_valid_dma_buffer_size()
{
  if constexpr (T::dma_enabled) {
    return T::stream_type::memory_buffer_type::size == T::regular_count;
  }
  else {
    return true;
  }
}

template<typename T>
consteval bool is_valid_dma_buffer_element()
{
  if constexpr (T::dma_enabled) {
    return std::same_as<
        typename T::stream_type::memory_buffer_type::element_type,
        std::uint16_t>;
  }
  else {
    return true;
  }
}

template<typename T>
consteval bool is_valid_dma_irq_priority()
{
  if constexpr (T::dma_enabled) {
    return std::convertible_to<decltype(T::dma_irq_priority),
                               nvic::irq_priority>;
  }
  else {
    return true;
  }
}

template<typename T>
consteval bool uses_single_buffer()
{
  if constexpr (T::dma_enabled) {
    return !T::stream_type::memory_buffer_type::double_buffer_mode;
  }
  else {
    return true;
  }
}

template<typename T>
consteval bool is_compatible_dma()
{
  if constexpr (T::dma_enabled) {
    return is_compatible_dma_stream<typename T::adc_instance,
                                    typename T::dma_stream>()
        && is_compatible_dma_channel<typename T::adc_instance,
                                     typename T::dma_channel>();
  }
  else {
    return true;
  }
}

} // namespace detail

template<typename T>
concept some_multi_channel_adc_traits = requires {
  requires some_adc_instance<typename T::adc_instance>;
  { T::injected_count } -> std::convertible_to<unsigned>;
  { T::regular_count } -> std::convertible_to<unsigned>;
  { T::dma_enabled } -> std::convertible_to<bool>;
  typename T::dma_stream;
  typename T::dma_channel;
  typename T::stream_type;
  { T::injected_trigger } -> std::convertible_to<std::optional<inj_trigger>>;
  { T::regular_trigger } -> std::convertible_to<std::optional<reg_trigger>>;
  { T::eoc_on_each } -> std::convertible_to<bool>;
  { T::auto_injconv } -> std::convertible_to<bool>;
};

template<some_multi_channel_adc_traits Traits, some_adc_channel... Channels>
class multi_channel_adc {
public:
  using adc_instance = Traits::adc_instance;
  using dma_stream = Traits::dma_stream;
  using dma_channel = Traits::dma_channel;
  using dma_stream_type = Traits::stream_type;
  using channels = emb::typelist<Channels...>;
  static constexpr unsigned injected_count = Traits::injected_count;
  static constexpr unsigned regular_count = Traits::regular_count;
  static constexpr bool dma_enabled = Traits::dma_enabled;
  static constexpr nvic::irq_priority dma_irq_priority =
      Traits::dma_irq_priority;
  static constexpr std::optional<inj_trigger> injected_trigger =
      Traits::injected_trigger;
  static constexpr std::optional<reg_trigger> regular_trigger =
      Traits::regular_trigger;
  static constexpr bool eoc_on_each = Traits::eoc_on_each;
  static constexpr bool auto_injconv = Traits::auto_injconv;

  static_assert(detail::ranks_cover_exactly<true, Channels...>(injected_count),
                "injected channel ranks must cover 1..injected_count exactly "
                "(no gaps, duplicates, or out-of-range positions)");
  static_assert(detail::ranks_cover_exactly<false, Channels...>(regular_count),
                "regular channel ranks must cover 1..regular_count exactly "
                "(no gaps, duplicates, or out-of-range positions)");
  static_assert(
      detail::is_compatible_dma<Traits>(),
      "dma_stream and dma_channel must be wired to this ADC instance");
  static_assert(detail::is_valid_dma_buffer_size<Traits>(),
                "DMA buffer size must equal regular_count");
  static_assert(
      detail::is_valid_dma_buffer_element<Traits>(),
      "DMA buffer element type must be uint16_t (regular data is 16-bit)");
  static_assert(detail::is_valid_dma_irq_priority<Traits>(),
                "dma_irq_priority must be convertible to nvic::irq_priority");
  static_assert(detail::uses_single_buffer<Traits>(),
                "multi_channel_adc requires a single-buffer DMA stream");
private:
  static inline registers& reg = adc_instance::reg;
  dma_stream_type dma_stream_;
  std::array<std::optional<gpio::analog_pin>, sizeof...(Channels)> pins_;
public:
  multi_channel_adc(multi_channel_adc const&) = delete;
  multi_channel_adc& operator=(multi_channel_adc const&) = delete;
  multi_channel_adc(multi_channel_adc&&) = delete;
  multi_channel_adc& operator=(multi_channel_adc&&) = delete;

  multi_channel_adc()
    requires(!dma_enabled)
  {
    adc_instance::enable_clock();
    detail::init_sequence(reg, get_config());
    init_channels();
  }

  multi_channel_adc()
    requires(dma_enabled)
      : dma_stream_(
            dma::peripheral_to_memory_stream_config{
                .irq_priority = dma_irq_priority},
            &reg.REGDATA)
  {
    adc_instance::enable_clock();
    detail::init_sequence(reg, get_config());
    init_channels();
  }

public:
  void enable()
  {
    if constexpr (dma_enabled) {
      dma_stream_.enable();
    }
    nvic::set_irq_priority(adc_instance::irqn, common_irq_priority);
    nvic::enable_irq(adc_instance::irqn);
  }

  void start_injected()
    requires(injected_count > 0 && !injected_trigger && !auto_injconv)
  {
    emb::mmio::set<ADC_CTRL2_INJSWSC>(reg.CTRL2);
  }

  void start_regular()
    requires(regular_count > 0 && !regular_trigger)
  {
    emb::mmio::set<ADC_CTRL2_REGSWSC>(reg.CTRL2);
  }

  template<unsigned Channel>
    requires(1 <= Channel && Channel <= injected_count)
  [[nodiscard]] std::uint16_t injected_result() const
  {
    if constexpr (Channel == 1)
      return static_cast<std::uint16_t>(reg.INJDATA1);
    else if constexpr (Channel == 2)
      return static_cast<std::uint16_t>(reg.INJDATA2);
    else if constexpr (Channel == 3)
      return static_cast<std::uint16_t>(reg.INJDATA3);
    else if constexpr (Channel == 4)
      return static_cast<std::uint16_t>(reg.INJDATA4);
  }

  template<unsigned Rank>
    requires(1 <= Rank && Rank <= regular_count && dma_enabled)
  [[nodiscard]] std::uint16_t regular_result() const
  {
    // DMA writes the buffer concurrently: read the slot through a volatile
    // reference so the load is not cached or elided
    auto const volatile& slot = dma_stream_.data().data[Rank - 1];
    return slot;
  }

  template<some_adc_channel Channel>
    requires(Channel::ranks.size() == 1)
  [[nodiscard]] std::uint16_t read(Channel) const
  {
    return result<Channel, Channel::ranks[0]>();
  }

  template<some_adc_channel Channel>
  [[nodiscard]] std::uint16_t oversample(Channel) const
  {
    constexpr std::size_t n = Channel::ranks.size();
    std::uint32_t sum = 0;
    emb::unroll<n>([&]<std::size_t I>() {
      sum += std::uint32_t{result<Channel, Channel::ranks[I]>()};
    });
    return static_cast<std::uint16_t>((sum + n / 2) / n);
  }

  template<some_adc_channel... Cs>
    requires(sizeof...(Cs) > 0)
  [[nodiscard]] std::array<std::uint16_t, sizeof...(Cs)>
  read_frame(Cs... chs) const
  {
    return {read(chs)...};
  }

  template<some_adc_channel... Cs>
  [[nodiscard]] auto read_frame(emb::typelist<Cs...>) const
  {
    return read_frame(Cs{}...);
  }

  template<some_adc_channel... Cs>
    requires(sizeof...(Cs) > 0)
  [[nodiscard]] std::array<std::uint16_t, sizeof...(Cs)>
  oversample_frame(Cs... chs) const
  {
    return {oversample(chs)...};
  }

  template<some_adc_channel... Cs>
  [[nodiscard]] auto oversample_frame(emb::typelist<Cs...>) const
  {
    return oversample_frame(Cs{}...);
  }
private:
  template<some_adc_channel Channel, unsigned Rank>
  [[nodiscard]] std::uint16_t result() const
  {
    static_assert(emb::typelist_contains_v<channels, Channel>,
                  "channel is not part of this ADC's conversion sequence");
    if constexpr (Channel::injected) {
      return injected_result<Rank>();
    }
    else {
      return regular_result<Rank>();
    }
  }

  void init_channels()
  {
    [[maybe_unused]] std::size_t i = 0;
    (
        [&] {
          std::optional<gpio::analog_pin_config> conf;
          if constexpr (requires { Channels::init(reg, injected_count); }) {
            conf = Channels::init(reg, injected_count); // injected channel
          }
          else {
            conf = Channels::init(reg); // regular channel
          }
          if (conf) {
            pins_[i].emplace(*conf);
          }
          ++i;
        }(),
        ...);
  }

  detail::sequence_config get_config() const
  {
    return detail::sequence_config{
        .injected_count = injected_count,
        .regular_count = regular_count,
        .dma_enabled = dma_enabled,
        .injected_trigger = injected_trigger,
        .regular_trigger = regular_trigger,
        .eoc_on_each_conversion = eoc_on_each,
        .auto_injected_conversion = auto_injconv,
    };
  }
};

template<typename T>
inline constexpr bool is_multi_channel_adc = false;

template<some_multi_channel_adc_traits Traits, some_adc_channel... Channels>
inline constexpr bool
    is_multi_channel_adc<multi_channel_adc<Traits, Channels...>> = true;

template<typename T>
concept some_multi_channel_adc = is_multi_channel_adc<T>;

} // namespace apm32::f4::adc
