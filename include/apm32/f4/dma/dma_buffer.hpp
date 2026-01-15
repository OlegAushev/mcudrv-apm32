#pragma once

#include <emb/meta.hpp>

#include <array>
#include <cstdint>

namespace apm32 {
namespace f4 {
namespace dma {
namespace v2 {

template<typename T>
concept dma_data_type = emb::same_as_any<T, uint8_t, uint16_t, uint32_t>;

template<dma_data_type ElementType, size_t Size>
struct memory_buffer {
  using element_type = ElementType;
  static constexpr bool double_buffer_mode = false;
  static constexpr size_t size = Size;
  std::array<element_type, Size> data __attribute__((aligned(32)));
};

template<std::unsigned_integral ElementType, size_t Size>
struct memory_double_buffer {
  using element_type = ElementType;
  static constexpr bool double_buffer_mode = true;
  static constexpr size_t size = Size;
  std::array<element_type, Size> data1 __attribute__((aligned(32)));
  std::array<element_type, Size> data2 __attribute__((aligned(32)));
};

} // namespace v2
} // namespace dma
} // namespace f4
} // namespace apm32
