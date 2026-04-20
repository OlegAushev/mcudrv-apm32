#pragma once

#include <apm32/f4/dma/dma_buffer.hpp>
#include <apm32/f4/dma/dma_channels.hpp>
#include <apm32/f4/dma/dma_controllers.hpp>
#include <apm32/f4/dma/dma_streams.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace dma {

struct peripheral_to_memory_stream_config {
  nvic::irq_priority irq_priority;
};

namespace detail {
void init_pm_stream(stream_registers& STREAM_REG, uint32_t ch);
} // namespace detail

template<
    some_dma_stream_instance Stream,
    some_dma_channel_instance Channel,
    typename MemoryBuffer>
class peripheral_to_memory_stream {
public:
  using controller_instance = Stream::controller;
  using stream_instance = Stream;
  using channel_instance = Channel;
  using memory_buffer_type = MemoryBuffer;
private:
  static inline controller_registers& DMA_REG = controller_instance::REG;
  static inline stream_registers& STREAM_REG = stream_instance::REG;
  static constexpr nvic::irq_number const irqn_ = stream_instance::irqn;

  memory_buffer_type dest_;
public:
  peripheral_to_memory_stream(
      peripheral_to_memory_stream_config const& conf,
      uint32_t volatile* periph_addr
  ) {
    controller_instance::enable_clock();

    detail::init_pm_stream(STREAM_REG, channel_instance::idx);

    if constexpr (!memory_buffer_type::double_buffer_mode) {
      emb::mmio::clear(STREAM_REG.SCFG, DMA_SCFGx_DBM);
      STREAM_REG.NDATA = memory_buffer_type::size;
      STREAM_REG.M0ADDR = reinterpret_cast<uint32_t>(dest_.data.data());
    } else {
      emb::mmio::set(STREAM_REG.SCFG, DMA_SCFGx_DBM);
      STREAM_REG.NDATA = memory_buffer_type::size;
      STREAM_REG.M0ADDR = reinterpret_cast<uint32_t>(dest_.data1.data());
      STREAM_REG.M1ADDR = reinterpret_cast<uint32_t>(dest_.data2.data());
    }

    STREAM_REG.PADDR = reinterpret_cast<uint32_t>(periph_addr);

    // Interrupts configuration
    emb::mmio::set(STREAM_REG.SCFG,
        DMA_SCFGx_DMEIEN | DMA_SCFGx_TXEIEN | DMA_SCFGx_TXCIEN);
    set_irq_priority(stream_instance::irqn, conf.irq_priority);
  }

  memory_buffer_type const& data() const {
    return dest_;
  }

  void enable() {
    nvic::enable_irq(stream_instance::irqn);
    emb::mmio::set(STREAM_REG.SCFG, DMA_SCFGx_EN);
  }

  void ack_interrupt() {
    if constexpr (channel_instance::idx >= 4) {
      DMA_REG.HIFCLR |= get_interrupt_clear_mask();
    } else {
      DMA_REG.LIFCLR |= get_interrupt_clear_mask();
    }
  }
private:
  consteval uint32_t get_interrupt_clear_mask() {
    static constexpr uint32_t mask = 0b101100; // TCIF, TEIF, DMEIF
    static constexpr std::array<uint32_t, 4> offsets = {0, 6, 16, 22};
    size_t i = (channel_instance::idx >= 4) ? channel_instance::idx - 4 :
                                              channel_instance::idx;
    return mask << offsets[i];
  }
};

} // namespace dma
} // namespace f4
} // namespace apm32
