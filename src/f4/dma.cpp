#include <apm32/f4/dma.hpp>

namespace apm32 {
namespace f4 {
namespace dma {

stream::stream(config conf)
    : peripheral_type(conf.stream_id),
      id_(conf.stream_id),
      regs_(streams[std::to_underlying(conf.stream_id)]) {
  enable_clock(id_);
  DMA_Config(regs_, &conf.hal_config);
}

void stream::enable_clock(stream_id id) {
  switch (id) {
  case stream_id::dma1_stream0:
  case stream_id::dma1_stream1:
  case stream_id::dma1_stream2:
  case stream_id::dma1_stream3:
  case stream_id::dma1_stream4:
  case stream_id::dma1_stream5:
  case stream_id::dma1_stream6:
  case stream_id::dma1_stream7:
    if (_is_clock_enabled[0]) {
      return;
    }
    detail::enable_clock[0]();
    _is_clock_enabled[0] = true;
    break;
  case stream_id::dma2_stream0:
  case stream_id::dma2_stream1:
  case stream_id::dma2_stream2:
  case stream_id::dma2_stream3:
  case stream_id::dma2_stream4:
  case stream_id::dma2_stream5:
  case stream_id::dma2_stream6:
  case stream_id::dma2_stream7:
    if (_is_clock_enabled[1]) {
      return;
    }
    detail::enable_clock[1]();
    _is_clock_enabled[1] = true;
    break;
  }
}

void stream::configure_interrupts(
    uint32_t interrupt_bitset,
    nvic::irq_priority priority
) {
  if ((interrupt_bitset & DMA_INT_FEIFLG) == DMA_INT_FEIFLG) {
    regs_->FCTRL_B.FEIEN = 1;
  }

  if (interrupt_bitset != DMA_INT_FEIFLG) {
    set_bit(regs_->SCFG, interrupt_bitset & 0x1E);
  }

  set_irq_priority(detail::irq_numbers[std::to_underlying(id_)], priority);
}

} // namespace dma
} // namespace f4
} // namespace apm32
