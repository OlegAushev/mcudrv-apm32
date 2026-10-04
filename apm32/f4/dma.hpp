#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/nvic.hpp>

#include <emb/meta.hpp>
#include <emb/mmio.hpp>

#include <cstddef>

namespace apm32::f4::dma {

using controller_registers = DMA_TypeDef;

inline constexpr std::size_t controller_count = 2;

struct dma1 {
  static inline controller_registers& reg = *DMA1;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_AHB1CLKEN_DMA1EN>(RCM->AHB1CLKEN);
  };
};

struct dma2 {
  static inline controller_registers& reg = *DMA2;

  static constexpr auto enable_clock = []() {
    emb::mmio::set<RCM_AHB1CLKEN_DMA2EN>(RCM->AHB1CLKEN);
  };
};

template<typename T>
struct is_dma_controller_instance
    : std::bool_constant<emb::same_as_any<T, dma1, dma2>> {};

template<typename T>
concept some_dma_controller_instance = is_dma_controller_instance<T>::value;

using stream_registers = DMA_Stream_TypeDef;

inline constexpr std::size_t stream_count = 16;

struct dma1_stream0 {
  using controller = dma1;
  static constexpr unsigned idx = 0;
  static inline stream_registers& reg = *DMA1_Stream0;
  static constexpr nvic::irq_number irqn = DMA1_Stream0_IRQn;
};

struct dma1_stream1 {
  using controller = dma1;
  static constexpr unsigned idx = 1;
  static inline stream_registers& reg = *DMA1_Stream1;
  static constexpr nvic::irq_number irqn = DMA1_Stream1_IRQn;
};

struct dma1_stream2 {
  using controller = dma1;
  static constexpr unsigned idx = 2;
  static inline stream_registers& reg = *DMA1_Stream2;
  static constexpr nvic::irq_number irqn = DMA1_Stream2_IRQn;
};

struct dma1_stream3 {
  using controller = dma1;
  static constexpr unsigned idx = 3;
  static inline stream_registers& reg = *DMA1_Stream3;
  static constexpr nvic::irq_number irqn = DMA1_Stream3_IRQn;
};

struct dma1_stream4 {
  using controller = dma1;
  static constexpr unsigned idx = 4;
  static inline stream_registers& reg = *DMA1_Stream4;
  static constexpr nvic::irq_number irqn = DMA1_Stream4_IRQn;
};

struct dma1_stream5 {
  using controller = dma1;
  static constexpr unsigned idx = 5;
  static inline stream_registers& reg = *DMA1_Stream5;
  static constexpr nvic::irq_number irqn = DMA1_Stream5_IRQn;
};

struct dma1_stream6 {
  using controller = dma1;
  static constexpr unsigned idx = 6;
  static inline stream_registers& reg = *DMA1_Stream6;
  static constexpr nvic::irq_number irqn = DMA1_Stream6_IRQn;
};

struct dma1_stream7 {
  using controller = dma1;
  static constexpr unsigned idx = 7;
  static inline stream_registers& reg = *DMA1_Stream7;
  static constexpr nvic::irq_number irqn = DMA1_Stream7_IRQn;
};

struct dma2_stream0 {
  using controller = dma2;
  static constexpr unsigned idx = 0;
  static inline stream_registers& reg = *DMA2_Stream0;
  static constexpr nvic::irq_number irqn = DMA2_Stream0_IRQn;
};

struct dma2_stream1 {
  using controller = dma2;
  static constexpr unsigned idx = 1;
  static inline stream_registers& reg = *DMA2_Stream1;
  static constexpr nvic::irq_number irqn = DMA2_Stream1_IRQn;
};

struct dma2_stream2 {
  using controller = dma2;
  static constexpr unsigned idx = 2;
  static inline stream_registers& reg = *DMA2_Stream2;
  static constexpr nvic::irq_number irqn = DMA2_Stream2_IRQn;
};

struct dma2_stream3 {
  using controller = dma2;
  static constexpr unsigned idx = 3;
  static inline stream_registers& reg = *DMA2_Stream3;
  static constexpr nvic::irq_number irqn = DMA2_Stream3_IRQn;
};

struct dma2_stream4 {
  using controller = dma2;
  static constexpr unsigned idx = 4;
  static inline stream_registers& reg = *DMA2_Stream4;
  static constexpr nvic::irq_number irqn = DMA2_Stream4_IRQn;
};

struct dma2_stream5 {
  using controller = dma2;
  static constexpr unsigned idx = 5;
  static inline stream_registers& reg = *DMA2_Stream5;
  static constexpr nvic::irq_number irqn = DMA2_Stream5_IRQn;
};

struct dma2_stream6 {
  using controller = dma2;
  static constexpr unsigned idx = 6;
  static inline stream_registers& reg = *DMA2_Stream6;
  static constexpr nvic::irq_number irqn = DMA2_Stream6_IRQn;
};

struct dma2_stream7 {
  using controller = dma2;
  static constexpr unsigned idx = 7;
  static inline stream_registers& reg = *DMA2_Stream7;
  static constexpr nvic::irq_number irqn = DMA2_Stream7_IRQn;
};

template<typename T>
struct is_dma_stream_instance
    : std::bool_constant<emb::same_as_any<T,
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
concept some_dma_stream_instance = is_dma_stream_instance<T>::value;

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
struct is_dma_channel_instance
    : std::bool_constant<emb::same_as_any<T,
                                          channel0,
                                          channel1,
                                          channel2,
                                          channel3,
                                          channel4,
                                          channel5,
                                          channel6,
                                          channel7>> {};

template<typename T>
concept some_dma_channel_instance = is_dma_channel_instance<T>::value;

} // namespace apm32::f4::dma
