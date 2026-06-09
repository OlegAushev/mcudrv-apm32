#pragma once

#include <apm32/f4/adc/adc.hpp>
#include <apm32/f4/adc/common_adc.hpp>
#include <apm32/f4/dma/pm_stream.hpp>
#include <apm32/f4/gpio/analog_pin.hpp>

#include <emb/mmio.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace apm32::f4::adc {

namespace detail {

template<typename T>
consteval bool is_valid_dma_buffer_size() {
  if constexpr (T::dma_enabled) {
    return T::stream_type::memory_buffer_type::size == T::regular_count;
  } else {
    return true;
  }
}

template<typename T>
consteval bool is_valid_dma_irq_priority() {
  if constexpr (T::dma_enabled) {
    return std::convertible_to<
        decltype(T::dma_irq_priority),
        nvic::irq_priority>;
  } else {
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

  requires(
      T::dma_enabled ? dma::some_dma_stream_instance<typename T::dma_stream>
                     : true
  );

  requires(
      T::dma_enabled ? dma::some_dma_channel_instance<typename T::dma_channel>
                     : true
  );

  requires is_compatible_dma_stream<
               typename T::adc_instance,
               typename T::dma_stream>()
               || std::is_void_v<typename T::dma_stream>;

  requires is_compatible_dma_channel<
               typename T::adc_instance,
               typename T::dma_channel>()
               || std::is_void_v<typename T::dma_channel>;

  requires detail::is_valid_dma_buffer_size<T>();

  requires detail::is_valid_dma_irq_priority<T>();

  { T::injected_trigger } -> std::convertible_to<std::optional<inj_trigger>>;
  { T::regular_trigger } -> std::convertible_to<std::optional<reg_trigger>>;
  { T::eoc_on_each } -> std::convertible_to<bool>;
  { T::auto_injconv } -> std::convertible_to<bool>;
};

namespace detail {

struct milti_channel_adc_config {
  unsigned injected_count;
  unsigned regular_count;
  bool dma_enabled;
  std::optional<inj_trigger> injected_trigger;
  std::optional<reg_trigger> regular_trigger;
  bool eoc_on_each_conversion;
  bool auto_injected_conversion;
};

void init_multi_channel_adc(
    registers& reg,
    detail::milti_channel_adc_config const& conf
);

} // namespace detail

template<some_multi_channel_adc_traits Traits, typename... Channels>
class multi_channel_adc {
public:
  using adc_instance = Traits::adc_instance;
  using dma_stream = Traits::dma_stream;
  using dma_channel = Traits::dma_channel;
  using dma_stream_type = Traits::stream_type;
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
    requires(!dma_enabled) {
    adc_instance::enable_clock();
    detail::init_multi_channel_adc(reg, get_config());
    init_channels();
  }

  multi_channel_adc()
    requires(dma_enabled)
      : dma_stream_(
            dma::peripheral_to_memory_stream_config{
                .irq_priority = dma_irq_priority
            },
            &reg.REGDATA
        ) {
    adc_instance::enable_clock();
    detail::init_multi_channel_adc(reg, get_config());
    init_channels();
  }

public:
  void enable() {
    if constexpr (dma_enabled) {
      dma_stream_.enable();
    }
    nvic::set_irq_priority(adc_instance::irqn, common_irq_priority);
    nvic::enable_irq(adc_instance::irqn);
  }

  void start_injected()
    requires(injected_count > 0 && !injected_trigger && !auto_injconv) {
    emb::mmio::set(reg.CTRL2, ADC_CTRL2_INJSWSC);
  }

  void start_regular()
    requires(regular_count > 0 && !regular_trigger) {
    emb::mmio::set(reg.CTRL2, ADC_CTRL2_REGSWSC);
  }

  template<unsigned Channel>
    requires(1 <= Channel && Channel <= injected_count)
  [[nodiscard]] std::uint32_t injected_result() const {
    if constexpr (Channel == 1) return reg.INJDATA1;
    else if constexpr (Channel == 2) return reg.INJDATA2;
    else if constexpr (Channel == 3) return reg.INJDATA3;
    else if constexpr (Channel == 4) return reg.INJDATA4;
  }

  template<unsigned Channel>
    requires(1 <= Channel && Channel <= injected_count)
  [[nodiscard]] std::uint32_t const volatile* injected_storage() const {
    if constexpr (Channel == 1) return &reg.INJDATA1;
    else if constexpr (Channel == 2) return &reg.INJDATA2;
    else if constexpr (Channel == 3) return &reg.INJDATA3;
    else if constexpr (Channel == 4) return &reg.INJDATA4;
  }

  template<unsigned Rank>
    requires(1 <= Rank && Rank <= regular_count && dma_enabled)
  [[nodiscard]] std::uint32_t const volatile* regular_storage() const {
    return &dma_stream_.data().data[Rank - 1];
  }
private:
  void init_channels() {
    [[maybe_unused]] std::size_t i = 0;
    (
        [&] {
          if (auto conf = Channels::init(reg, injected_count)) {
            pins_[i].emplace(*conf);
          }
          ++i;
        }(),
        ...
    );
  }

  detail::milti_channel_adc_config get_config() const {
    return detail::milti_channel_adc_config{
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

} // namespace apm32::f4::adc
