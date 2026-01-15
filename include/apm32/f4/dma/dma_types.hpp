#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_dma.h>
#include <apm32f4xx_rcm.h>

#include <concepts>

namespace apm32 {
namespace f4 {
namespace dma {
namespace v2 {

using controller_registers = DMA_T;
using stream_registers = DMA_Stream_T;

inline constexpr size_t controller_count = 2;
inline constexpr size_t stream_count = 16;

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

struct dma1_stream0 {
  using controller = dma1;
  static constexpr unsigned idx = 0;
  static inline stream_registers& regs = *DMA1_Stream0;
  static constexpr nvic::irq_number irqn = DMA1_STR0_IRQn;
};

struct dma1_stream1 {
  using controller = dma1;
  static constexpr unsigned idx = 1;
  static inline stream_registers& regs = *DMA1_Stream1;
  static constexpr nvic::irq_number irqn = DMA1_STR1_IRQn;
};

struct dma1_stream2 {
  using controller = dma1;
  static constexpr unsigned idx = 2;
  static inline stream_registers& regs = *DMA1_Stream2;
  static constexpr nvic::irq_number irqn = DMA1_STR2_IRQn;
};

struct dma1_stream3 {
  using controller = dma1;
  static constexpr unsigned idx = 3;
  static inline stream_registers& regs = *DMA1_Stream3;
  static constexpr nvic::irq_number irqn = DMA1_STR3_IRQn;
};

struct dma1_stream4 {
  using controller = dma1;
  static constexpr unsigned idx = 4;
  static inline stream_registers& regs = *DMA1_Stream4;
  static constexpr nvic::irq_number irqn = DMA1_STR4_IRQn;
};

struct dma1_stream5 {
  using controller = dma1;
  static constexpr unsigned idx = 5;
  static inline stream_registers& regs = *DMA1_Stream5;
  static constexpr nvic::irq_number irqn = DMA1_STR5_IRQn;
};

struct dma1_stream6 {
  using controller = dma1;
  static constexpr unsigned idx = 6;
  static inline stream_registers& regs = *DMA1_Stream6;
  static constexpr nvic::irq_number irqn = DMA1_STR6_IRQn;
};

struct dma1_stream7 {
  using controller = dma1;
  static constexpr unsigned idx = 7;
  static inline stream_registers& regs = *DMA1_Stream7;
  static constexpr nvic::irq_number irqn = DMA1_STR7_IRQn;
};

struct dma2_stream0 {
  using controller = dma2;
  static constexpr unsigned idx = 0;
  static inline stream_registers& regs = *DMA2_Stream0;
  static constexpr nvic::irq_number irqn = DMA2_STR0_IRQn;
};

struct dma2_stream1 {
  using controller = dma2;
  static constexpr unsigned idx = 1;
  static inline stream_registers& regs = *DMA2_Stream1;
  static constexpr nvic::irq_number irqn = DMA2_STR1_IRQn;
};

struct dma2_stream2 {
  using controller = dma2;
  static constexpr unsigned idx = 2;
  static inline stream_registers& regs = *DMA2_Stream2;
  static constexpr nvic::irq_number irqn = DMA2_STR2_IRQn;
};

struct dma2_stream3 {
  using controller = dma2;
  static constexpr unsigned idx = 3;
  static inline stream_registers& regs = *DMA2_Stream3;
  static constexpr nvic::irq_number irqn = DMA2_STR3_IRQn;
};

struct dma2_stream4 {
  using controller = dma2;
  static constexpr unsigned idx = 4;
  static inline stream_registers& regs = *DMA2_Stream4;
  static constexpr nvic::irq_number irqn = DMA2_STR4_IRQn;
};

struct dma2_stream5 {
  using controller = dma2;
  static constexpr unsigned idx = 5;
  static inline stream_registers& regs = *DMA2_Stream5;
  static constexpr nvic::irq_number irqn = DMA2_STR5_IRQn;
};

struct dma2_stream6 {
  using controller = dma2;
  static constexpr unsigned idx = 6;
  static inline stream_registers& regs = *DMA2_Stream6;
  static constexpr nvic::irq_number irqn = DMA2_STR6_IRQn;
};

struct dma2_stream7 {
  using controller = dma2;
  static constexpr unsigned idx = 7;
  static inline stream_registers& regs = *DMA2_Stream7;
  static constexpr nvic::irq_number irqn = DMA2_STR7_IRQn;
};

template<typename T>
struct is_dma_stream_instance : std::bool_constant<emb::same_as_any<
                                    T,
                                    dma1_stream0,
                                    dma1_stream1,
                                    dma1_stream2,
                                    dma1_stream3,
                                    dma1_stream4,
                                    dma1_stream5,
                                    dma1_stream6,
                                    dma1_stream7,
                                    dma2_stream0,
                                    dma2_stream1,
                                    dma2_stream2,
                                    dma2_stream3,
                                    dma2_stream4,
                                    dma2_stream5,
                                    dma2_stream6,
                                    dma2_stream7>> {};

template<typename T>
concept dma_stream_instance = is_dma_stream_instance<T>::value;

struct channel0 {
  static constexpr unsigned idx = 0;
};

struct channel1 {
  static constexpr unsigned idx = 1;
};

struct channel2 {
  static constexpr unsigned idx = 2;
};

struct channel3 {
  static constexpr unsigned idx = 3;
};

struct channel4 {
  static constexpr unsigned idx = 4;
};

struct channel5 {
  static constexpr unsigned idx = 5;
};

struct channel6 {
  static constexpr unsigned idx = 6;
};

struct channel7 {
  static constexpr unsigned idx = 7;
};

template<typename T>
struct is_dma_channel_instance : std::bool_constant<emb::same_as_any<
                                     T,
                                     channel0,
                                     channel1,
                                     channel2,
                                     channel3,
                                     channel4,
                                     channel5,
                                     channel6,
                                     channel7>> {};

template<typename T>
concept dma_channel_instance = is_dma_channel_instance<T>::value;

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
