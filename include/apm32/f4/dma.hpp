#pragma once

#include <apm32/f4/dma/pm_stream.hpp>

// TODO delete all below

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_dma.h>
#include <apm32f4xx_rcm.h>

#include <array>
#include <utility>

namespace apm32 {
namespace f4 {
namespace dma {

using stream_registers = DMA_Stream_T;

inline constexpr size_t stream_count = 16;

inline std::array<stream_registers*, stream_count> const streams = {
    DMA1_Stream0,
    DMA1_Stream1,
    DMA1_Stream2,
    DMA1_Stream3,
    DMA1_Stream4,
    DMA1_Stream5,
    DMA1_Stream6,
    DMA1_Stream7,
    DMA2_Stream0,
    DMA2_Stream1,
    DMA2_Stream2,
    DMA2_Stream3,
    DMA2_Stream4,
    DMA2_Stream5,
    DMA2_Stream6,
    DMA2_Stream7
};

enum class stream_id : uint32_t {
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
  dma2_stream7
};

struct config {
  apm32::f4::dma::stream_id stream_id;
  DMA_Config_T hal_config;
};

namespace detail {

inline std::array<void (*)(void), 2> const enable_clock = {
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA1); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA2); },
};

inline constexpr std::array<nvic::irq_number, stream_count> irq_numbers = {
    DMA1_STR0_IRQn,
    DMA1_STR1_IRQn,
    DMA1_STR2_IRQn,
    DMA1_STR3_IRQn,
    DMA1_STR4_IRQn,
    DMA1_STR5_IRQn,
    DMA1_STR6_IRQn,
    DMA1_STR7_IRQn,
    DMA2_STR0_IRQn,
    DMA2_STR1_IRQn,
    DMA2_STR2_IRQn,
    DMA2_STR3_IRQn,
    DMA2_STR4_IRQn,
    DMA2_STR5_IRQn,
    DMA2_STR6_IRQn,
    DMA2_STR7_IRQn
};

} // namespace detail

class stream : public core::peripheral<stream, stream_id, stream_count> {
public:
  using peripheral_type = core::peripheral<stream, stream_id, stream_count>;
private:
  stream_id const id_;
  stream_registers* const regs_;
  static inline std::array<bool, 2> _is_clock_enabled{};
public:
  stream(config conf);

  stream_registers* regs() {
    return regs_;
  }

  void configure_interrupts(
      uint32_t interrupt_bitset,
      nvic::irq_priority priority
  );

  void enable_interrupts() {
    nvic::enable_irq(detail::irq_numbers[std::to_underlying(id_)]);
  }

  void disable_interrupts() {
    nvic::disable_irq(detail::irq_numbers[std::to_underlying(id_)]);
  }

  void enable() {
    regs_->SCFG_B.EN = 1;
  }
private:
  static void enable_clock(stream_id id);
};

template<typename T, size_t Size>
class memory_buffer {
public:
  using value_type = T;
  using size_type = size_t;
  using reference = value_type&;
  using const_reference = value_type const&;
  using pointer = value_type*;
  using const_pointer = value_type const*;
private:
  std::array<T, Size> data_ __attribute__((aligned(32)));
  stream& stream_;
public:
  memory_buffer(stream& s) : stream_(s) {
    stream_.regs()->SCFG_B.DBM = 0;
    stream_.regs()->NDATA = static_cast<uint32_t>(data_.size());
    stream_.regs()->M0ADDR = reinterpret_cast<uint32_t>(data_.data());
  }

  const_pointer data() const {
    return data_.data();
  }

  constexpr size_type size() const {
    return data_.size();
  }

  reference operator[](size_t pos) {
    return data_[pos];
  }

  const_reference operator[](size_t pos) const {
    return data_[pos];
  }
};

template<typename T, size_t Size>
class memory_double_buffer {
public:
  using value_type = T;
  using size_type = size_t;
  using reference = value_type&;
  using const_reference = value_type const&;
  using pointer = value_type*;
  using const_pointer = value_type const*;
private:
  stream& stream_;
public:
  memory_buffer<T, Size> buf0;
  memory_buffer<T, Size> buf1;
public:
  memory_double_buffer(stream& s) : stream_(s), buf0(s), buf1(s) {
    stream_.regs()->SCFG_B.DBM = 1;
    stream_.regs()->NDATA = static_cast<uint32_t>(Size);
    stream_.regs()->M0ADDR = static_cast<uint32_t>(buf0.data());
    stream_.regs()->M1ADDR = reinterpret_cast<uint32_t>(buf1.data());
  }
};

} // namespace dma
} // namespace f4
} // namespace apm32
