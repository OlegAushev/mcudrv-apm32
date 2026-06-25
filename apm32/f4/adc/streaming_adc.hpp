#pragma once

#include <apm32/f4/adc/adc.hpp>
#include <apm32/f4/adc/adc_sequence.hpp>
#include <apm32/f4/adc/common_adc.hpp>
#include <apm32/f4/dma/pm_stream.hpp>
#include <apm32/f4/gpio/analog_pin.hpp>

#include <emb/mmio.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <span>

namespace apm32::f4::adc {

namespace detail {

template<typename T>
consteval bool is_valid_stream_buffer() {
  using buf = typename T::stream_type::memory_buffer_type;
  return buf::double_buffer_mode && (buf::size % T::regular_count == 0);
}

} // namespace detail

// Traits for a DMA-only, double-buffered regular ADC. Unlike
// some_multi_channel_adc_traits there are no injected channels and no
// per-element access: the DMA continuously fills a double buffer and the CPU
// reads a whole completed window. Conversions are driven by regular_trigger.
template<typename T>
concept some_streaming_adc_traits = requires {
  requires some_adc_instance<typename T::adc_instance>;
  { T::regular_count } -> std::convertible_to<unsigned>;
  typename T::dma_stream;
  typename T::dma_channel;
  typename T::stream_type;
  { T::regular_trigger } -> std::convertible_to<reg_trigger>;
  { T::dma_irq_priority } -> std::convertible_to<nvic::irq_priority>;
  { T::eoc_on_each } -> std::convertible_to<bool>;
};

template<some_streaming_adc_traits Traits, some_adc_channel... Channels>
class streaming_adc {
public:
  using adc_instance = Traits::adc_instance;
  using dma_stream = Traits::dma_stream;
  using dma_channel = Traits::dma_channel;
  using dma_stream_type = Traits::stream_type;
  using element_type =
      typename dma_stream_type::memory_buffer_type::element_type;

  static constexpr unsigned regular_count = Traits::regular_count;
  static constexpr std::size_t buffer_size =
      dma_stream_type::memory_buffer_type::size;
  static constexpr std::size_t frames = buffer_size / regular_count;
  static constexpr reg_trigger regular_trigger = Traits::regular_trigger;
  static constexpr nvic::irq_priority dma_irq_priority =
      Traits::dma_irq_priority;
  static constexpr bool eoc_on_each = Traits::eoc_on_each;

  static_assert(
      ((!Channels::injected) && ...),
      "streaming_adc supports regular channels only"
  );
  static_assert(
      detail::ranks_cover_exactly<false, Channels...>(regular_count),
      "regular channel ranks must cover 1..regular_count exactly "
      "(no gaps, duplicates, or out-of-range positions)"
  );
  static_assert(
      is_compatible_dma_stream<adc_instance, dma_stream>(),
      "dma_stream is not wired to this ADC instance"
  );
  static_assert(
      is_compatible_dma_channel<adc_instance, dma_channel>(),
      "dma_channel is not the DMA request channel for this ADC instance"
  );
  static_assert(
      detail::is_valid_stream_buffer<Traits>(),
      "streaming_adc needs a double buffer whose size is a multiple of "
      "regular_count"
  );
private:
  static inline registers& reg = adc_instance::reg;
  dma_stream_type dma_stream_;
  std::array<std::optional<gpio::analog_pin>, sizeof...(Channels)> pins_;
  std::atomic<bool> completed_ = false;
public:
  streaming_adc(streaming_adc const&) = delete;
  streaming_adc& operator=(streaming_adc const&) = delete;
  streaming_adc(streaming_adc&&) = delete;
  streaming_adc& operator=(streaming_adc&&) = delete;

  streaming_adc()
      : dma_stream_(
            dma::peripheral_to_memory_stream_config{
                .irq_priority = dma_irq_priority
            },
            &reg.REGDATA
        ) {
    adc_instance::enable_clock();
    detail::init_sequence(reg, get_config());
    init_channels();
  }

  void enable() {
    dma_stream_.enable();
    if constexpr (eoc_on_each) {
      nvic::set_irq_priority(adc_instance::irqn, common_irq_priority);
      nvic::enable_irq(adc_instance::irqn);
    }
  }

  // Call from the DMA stream's transfer-complete ISR.
  void on_dma_complete() {
    dma_stream_.ack_interrupt();
    completed_.store(true, std::memory_order::release);
  }

  // Returns the just-completed window once it was filled, else nullopt.
  [[nodiscard]] std::optional<std::span<element_type const>> completed() const {
    if (!completed_.load(std::memory_order::acquire)) {
      return std::nullopt;
    }
    return dma_stream_.completed_buffer();
  }

  void consume() {
    completed_.store(false, std::memory_order::release);
  }
private:
  void init_channels() {
    [[maybe_unused]] std::size_t i = 0;
    (
        [&] {
          std::optional<gpio::analog_pin_config> conf = Channels::init(reg);
          if (conf) {
            pins_[i].emplace(*conf);
          }
          ++i;
        }(),
        ...
    );
  }

  detail::sequence_config get_config() const {
    return detail::sequence_config{
        .injected_count = 0,
        .regular_count = regular_count,
        .dma_enabled = true,
        .injected_trigger = std::nullopt,
        .regular_trigger = regular_trigger,
        .eoc_on_each_conversion = eoc_on_each,
        .auto_injected_conversion = false,
    };
  }
};

} // namespace apm32::f4::adc
