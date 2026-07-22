#pragma once

#include <apm32/f4/dma/dma.hpp>

#include <emb/mmio.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

namespace apm32::f4::dma {

struct peripheral_to_memory_stream_config {
  nvic::irq_priority irq_priority;
};

namespace detail {
// DMA PSIZE/MSIZE field encoding: 0b00 byte, 0b01 half-word, 0b10 word.
template<dma_data_type T>
consteval std::uint32_t dma_size_cfg() {
  if constexpr (sizeof(T) == 1) {
    return 0b00u;
  } else if constexpr (sizeof(T) == 2) {
    return 0b01u;
  } else {
    return 0b10u;
  }
}
} // namespace detail

template<
    some_dma_stream_instance Stream,
    some_dma_channel_instance Channel,
    typename Storage>
class peripheral_to_memory_stream {
public:
  using controller_instance = Stream::controller;
  using stream_instance = Stream;
  using channel_instance = Channel;
  using storage_type = Storage;
  using memory_buffer_type = typename Storage::buffer_type;
private:
  static inline controller_registers& DMA_REG = controller_instance::REG;
  static inline stream_registers& STREAM_REG = stream_instance::REG;
  static constexpr nvic::irq_number const irqn_ = stream_instance::irqn;

  Storage storage_;
public:
  peripheral_to_memory_stream(peripheral_to_memory_stream const&) = delete;
  peripheral_to_memory_stream&
  operator=(peripheral_to_memory_stream const&) = delete;
  peripheral_to_memory_stream(peripheral_to_memory_stream&&) = delete;
  peripheral_to_memory_stream&
  operator=(peripheral_to_memory_stream&&) = delete;

  peripheral_to_memory_stream(
      peripheral_to_memory_stream_config const& conf,
      std::uint32_t volatile* periph_addr
  ) {
    controller_instance::enable_clock();

    emb::mmio::modify(
        STREAM_REG.SCFG,
        emb::mmio::bits<DMA_SCFGx_CHSEL>(channel_instance::idx),
        emb::mmio::bits<DMA_SCFGx_DIRCFG>(0b00u), // periph to memory
        emb::mmio::bits<DMA_SCFGx_CIRCMEN>(1u),   // circular mode
        emb::mmio::bits<DMA_SCFGx_PERIM>(0u),     // no periph increment
        emb::mmio::bits<DMA_SCFGx_MEMIM>(1u),     // memory increment
        emb::mmio::bits<DMA_SCFGx_PERSIZECFG>(
            detail::dma_size_cfg<typename memory_buffer_type::element_type>()
        ),
        emb::mmio::bits<DMA_SCFGx_MEMSIZECFG>(
            detail::dma_size_cfg<typename memory_buffer_type::element_type>()
        ),
        emb::mmio::bits<DMA_SCFGx_PRILCFG>(0b10u) // high priority
    );

    if constexpr (!memory_buffer_type::double_buffer_mode) {
      emb::mmio::clear<DMA_SCFGx_DBM>(STREAM_REG.SCFG);
      STREAM_REG.NDATA = memory_buffer_type::size;
      STREAM_REG.M0ADDR = reinterpret_cast<std::uint32_t>(
          storage_.get().data.data()
      );
    } else {
      emb::mmio::set<DMA_SCFGx_DBM>(STREAM_REG.SCFG);
      STREAM_REG.NDATA = memory_buffer_type::size;
      STREAM_REG.M0ADDR = reinterpret_cast<std::uint32_t>(
          storage_.get().data1.data()
      );
      STREAM_REG.M1ADDR = reinterpret_cast<std::uint32_t>(
          storage_.get().data2.data()
      );
    }

    STREAM_REG.PADDR = reinterpret_cast<std::uint32_t>(periph_addr);

    // Interrupts configuration
    emb::mmio::set<DMA_SCFGx_DMEIEN | DMA_SCFGx_TXEIEN | DMA_SCFGx_TXCIEN>(
        STREAM_REG.SCFG
    );
    set_irq_priority(stream_instance::irqn, conf.irq_priority);
  }

  memory_buffer_type const& data() const {
    return storage_.get();
  }

  // Returns a view over the buffer the DMA is NOT currently writing (the just
  // completed half). The CT (current target) bit selects the active buffer:
  // CT=1 -> DMA writes M1 (data2) -> data1 is complete; CT=0 -> data2 complete.
  std::span<typename memory_buffer_type::element_type const>
  completed_buffer() const
    requires(memory_buffer_type::double_buffer_mode) {
    using element = typename memory_buffer_type::element_type;
    auto const& b = storage_.get();
    return emb::mmio::test_any(STREAM_REG.SCFG, DMA_SCFGx_CTARG)
               ? std::span<element const>{b.data1.data(), b.data1.size()}
               : std::span<element const>{b.data2.data(), b.data2.size()};
  }

  void enable() {
    nvic::enable_irq(stream_instance::irqn);
    emb::mmio::set<DMA_SCFGx_EN>(STREAM_REG.SCFG);
  }

  void ack_interrupt() {
    if constexpr (channel_instance::idx >= 4) {
      DMA_REG.HIFCLR |= get_interrupt_clear_mask();
    } else {
      DMA_REG.LIFCLR |= get_interrupt_clear_mask();
    }
  }
private:
  consteval std::uint32_t get_interrupt_clear_mask() {
    static constexpr std::uint32_t mask = 0b101100; // TCIF, TEIF, DMEIF
    static constexpr std::array<std::uint32_t, 4> offsets = {0, 6, 16, 22};
    std::size_t i = (channel_instance::idx >= 4) ? channel_instance::idx - 4
                                                 : channel_instance::idx;
    return mask << offsets[i];
  }
};

} // namespace apm32::f4::dma
