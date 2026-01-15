#pragma once

#include <apm32/f4/adc/adc_types.hpp>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

namespace traits {

template<typename T>
consteval bool check_dma_buffer_size() {
  if constexpr (T::dma_enabled) {
    return T::stream_type::memory_buffer_type::size == T::regular_count;
  } else {
    return true;
  }
}

template<typename T>
concept basic_traits = requires(T t) {
  { T::injected_count } -> std::convertible_to<unsigned>;
  { T::regular_count } -> std::convertible_to<unsigned>;
  { T::dma_enabled } -> std::convertible_to<bool>;
  requires(
      T::dma_enabled ? dma::v2::dma_stream_instance<typename T::dma_stream> :
                       true
  );
  requires(
      T::dma_enabled ? dma::v2::dma_channel_instance<typename T::dma_channel> :
                       true
  );
  requires check_dma_buffer_size<T>();
};

template<size_t InjectedCount, size_t RegularCount>
struct no_dma_policy {
  static constexpr unsigned injected_count = InjectedCount;
  static constexpr unsigned regular_count = RegularCount;
  static constexpr bool dma_enabled = false;
  struct dma_stream {};
  struct dma_channel {};
  struct stream_type {};
};

template<
    unsigned InjectedCount,
    unsigned RegularCount,
    dma::v2::dma_stream_instance DmaStream,
    dma::v2::dma_channel_instance DmaChannel>
  requires (InjectedCount > 0 && RegularCount > 0)
struct dma_policy {
  static constexpr unsigned injected_count = InjectedCount;
  static constexpr unsigned regular_count = RegularCount;
  static constexpr bool dma_enabled = true;
  using dma_stream = DmaStream;
  using dma_channel = DmaChannel;
  using stream_type = dma::v2::peripheral_to_memory_stream<
      dma_stream,
      dma_channel,
      dma::v2::memory_buffer<uint32_t, regular_count>>;
};

static_assert(basic_traits<no_dma_policy<1, 2>>);
static_assert(
    basic_traits<dma_policy<3, 5, dma::v2::dma2_stream2, dma::v2::channel4>>
);

} // namespace traits

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
