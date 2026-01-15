#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32f4xx_dma.h>
#include <apm32f4xx_rcm.h>

#include <emb/meta.hpp>

namespace apm32 {
namespace f4 {
namespace dma {
namespace v2 {

using controller_registers = DMA_T;

inline constexpr size_t controller_count = 2;

struct dma1 {
  static inline controller_registers& regs = *DMA1;

  static constexpr auto enable_clock = []() {
    RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA1);
  };
};

struct dma2 {
  static inline controller_registers& regs = *DMA2;

  static constexpr auto enable_clock = []() {
    RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA2);
  };
};

template<typename T>
struct is_dma_controller_instance
    : std::bool_constant<emb::same_as_any<T, dma1, dma2>> {};

template<typename T>
concept dma_controller_instance = is_dma_controller_instance<T>::value;

} // namespace v2
} // namespace dma
} // namespace f4
} // namespace apm32
