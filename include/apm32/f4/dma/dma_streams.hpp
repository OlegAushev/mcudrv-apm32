#pragma once

#include <apm32/f4/dma/dma_controllers.hpp>

#include <apm32/f4/nvic.hpp>

namespace apm32 {
namespace f4 {
namespace dma {
namespace v2 {

using stream_registers = DMA_Stream_T;

inline constexpr size_t stream_count = 16;

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

} // namespace v2
} // namespace dma
} // namespace f4
} // namespace apm32
