#pragma once

#include <apm32/f4/dma/dma_buffer.hpp>
#include <apm32/f4/dma/dma_channels.hpp>
#include <apm32/f4/dma/dma_controllers.hpp>
#include <apm32/f4/dma/dma_streams.hpp>

namespace apm32 {
namespace f4 {
namespace dma {
namespace v2 {

struct peripheral_to_memory_stream_config {
  nvic::irq_priority irq_priority;
};

namespace detail {
void init_pm_stream(stream_registers& regs, uint32_t ch);
} // namespace detail

template<
    dma_stream_instance Stream,
    dma_channel_instance Channel,
    typename MemoryBuffer>
class peripheral_to_memory_stream {
public:
  using controller_instance = Stream::controller;
  using stream_instance = Stream;
  using channel_instance = Channel;
  using memory_buffer_type = MemoryBuffer;
private:
  static inline controller_registers& controller_regs_ =
      controller_instance::regs;
  static inline stream_registers& stream_regs_ = stream_instance::regs;
  static inline nvic::irq_number const irqn_ = stream_instance::irqn;

  memory_buffer_type dest_;
public:
  peripheral_to_memory_stream(
      peripheral_to_memory_stream_config const& conf,
      uint32_t volatile* periph_addr
  ) {
    controller_instance::enable_clock();

    detail::init_pm_stream(stream_regs_, channel_instance::idx);

    if constexpr (!memory_buffer_type::double_buffer_mode) {
      stream_regs_.SCFG_B.DBM = 0;
      stream_regs_.NDATA = memory_buffer_type::size;
      stream_regs_.M0ADDR = reinterpret_cast<uint32_t>(dest_.data.data());
    } else {
      stream_regs_.SCFG_B.DBM = 1;
      stream_regs_.NDATA = memory_buffer_type::size;
      stream_regs_.M0ADDR = reinterpret_cast<uint32_t>(dest_.data1.data());
      stream_regs_.M1ADDR = reinterpret_cast<uint32_t>(dest_.data2.data());
    }

    stream_regs_.PADDR = reinterpret_cast<uint32_t>(periph_addr);

    // Interrupts configuration
    stream_regs_.SCFG_B.DMEIEN = 1;
    stream_regs_.SCFG_B.TXEIEN = 1;
    stream_regs_.SCFG_B.TXCIEN = 1;
    set_irq_priority(stream_instance::irqn, conf.irq_priority);
  }

  memory_buffer_type const& data() const {
    return dest_;
  }

  void enable() {
    nvic::enable_irq(stream_instance::irqn);
    stream_regs_.SCFG_B.EN = 1;
  }

  void ack_interrupt() {
    if constexpr (channel_instance::idx >= 4) {
      controller_regs_.HIFCLR |= get_interrupt_clear_mask();
    } else {
      controller_regs_.LIFCLR |= get_interrupt_clear_mask();
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

} // namespace v2
} // namespace dma
} // namespace f4
} // namespace apm32
