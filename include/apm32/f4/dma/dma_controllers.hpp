#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <emb/meta.hpp>
#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace dma {

using controller_registers = DMA_TypeDef;

inline constexpr size_t controller_count = 2;

struct dma1 {
  static inline controller_registers& regs = *DMA1;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DMA1EN);
  };
};

struct dma2 {
  static inline controller_registers& regs = *DMA2;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->AHB1CLKEN, RCM_AHB1CLKEN_DMA2EN);
  };
};

template<typename T>
struct is_dma_controller_instance
    : std::bool_constant<emb::same_as_any<T, dma1, dma2>> {};

template<typename T>
concept dma_controller_instance = is_dma_controller_instance<T>::value;

} // namespace dma
} // namespace f4
} // namespace apm32
