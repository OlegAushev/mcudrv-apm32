#pragma once

#include <apm32/f4/adc/adc_channels.hpp>
#include <apm32/f4/adc/adc_instances.hpp>
#include <apm32/f4/adc/adc_types.hpp>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

namespace traits {

namespace detail {

template<adc_module_instance Instance, typename T>
consteval bool check_dma_stream() {
  if constexpr (dma::v2::dma_stream_instance<T>) {
    return emb::typelist_contains_v<typename Instance::dma_streams, T>;
  } else if constexpr (std::is_void_v<T>) {
    return true;
  } else {
    return false;
  }
}

template<adc_module_instance Instance, typename T>
consteval bool check_dma_channel() {
  if constexpr (dma::v2::dma_channel_instance<T>) {
    return std::same_as<T, typename Instance::dma_channel>;
  } else if constexpr (std::is_void_v<T>) {
    return true;
  } else {
    return false;
  }
}

template<typename T>
consteval bool check_dma_buffer() {
  if constexpr (T::dma_enabled) {
    return T::stream_type::memory_buffer_type::size == T::regular_count;
  } else {
    return true;
  }
}

template<typename T>
consteval bool check_dma_irq_priority() {
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
concept basic_traits = requires {
  requires adc_module_instance<typename T::adc_instance>;

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

  requires detail::check_dma_stream<
      typename T::adc_instance,
      typename T::dma_stream>();
  requires detail::check_dma_channel<
      typename T::adc_instance,
      typename T::dma_channel>();
  requires detail::check_dma_buffer<T>();
  requires detail::check_dma_irq_priority<T>();

  { T::injected_trigger } -> std::convertible_to<std::optional<inj_trigger>>;
  { T::regular_trigger } -> std::convertible_to<std::optional<reg_trigger>>;
  { T::eoc_on_each } -> std::convertible_to<bool>;
  { T::auto_injconv } -> std::convertible_to<bool>;
};

} // namespace traits

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
