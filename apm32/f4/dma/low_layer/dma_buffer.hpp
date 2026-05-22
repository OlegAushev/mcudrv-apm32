#pragma once

#include <emb/meta.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace apm32::f4::dma {

template<typename T>
concept dma_data_type =
    emb::same_as_any<T, std::uint8_t, std::uint16_t, std::uint32_t>;

template<dma_data_type ElementType, std::size_t Size>
struct memory_buffer {
  using element_type = ElementType;
  static constexpr bool double_buffer_mode = false;
  static constexpr std::size_t size = Size;
  std::array<element_type, Size> data __attribute__((aligned(32)));
};

template<std::unsigned_integral ElementType, std::size_t Size>
struct memory_double_buffer {
  using element_type = ElementType;
  static constexpr bool double_buffer_mode = true;
  static constexpr std::size_t size = Size;
  std::array<element_type, Size> data1 __attribute__((aligned(32)));
  std::array<element_type, Size> data2 __attribute__((aligned(32)));
};

} // namespace apm32::f4::dma
