#pragma once

#include <apm32/device.hpp>

#include <emb/meta.hpp>
#include <emb/mmio.hpp>

#include <cstddef>

namespace apm32::f4::dma {

using controller_registers = DMA_TypeDef;

inline constexpr std::size_t controller_count = 2;

struct dma1 {
  static inline controller_registers& REG = *DMA1;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_AHB1CLKEN_DMA1EN>(RCM->AHB1CLKEN);
  };
};

struct dma2 {
  static inline controller_registers& REG = *DMA2;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_AHB1CLKEN_DMA2EN>(RCM->AHB1CLKEN);
  };
};

template<typename T>
struct is_dma_controller_instance
    : std::bool_constant<emb::same_as_any<T, dma1, dma2>> {};

template<typename T>
concept some_dma_controller_instance = is_dma_controller_instance<T>::value;

} // namespace apm32::f4::dma
