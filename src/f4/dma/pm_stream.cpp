#include <apm32/f4/dma/driver/pm_stream.hpp>

#include <emb/mmio.hpp>

namespace apm32 {
namespace f4 {
namespace dma {

void detail::init_pm_stream(stream_registers& STREAM_REG, uint32_t ch) {
  emb::mmio::modify(STREAM_REG.SCFG,
      emb::mmio::bits<DMA_SCFGx_CHSEL>(ch),
      emb::mmio::bits<DMA_SCFGx_DIRCFG>(0b00u),       // periph to memory
      emb::mmio::bits<DMA_SCFGx_CIRCMEN>(1u),         // circular mode
      emb::mmio::bits<DMA_SCFGx_PERIM>(0u),           // no periph increment
      emb::mmio::bits<DMA_SCFGx_MEMIM>(1u),           // memory increment
      emb::mmio::bits<DMA_SCFGx_PERSIZECFG>(0b10u),   // word (32-bit)
      emb::mmio::bits<DMA_SCFGx_MEMSIZECFG>(0b10u),   // word (32-bit)
      emb::mmio::bits<DMA_SCFGx_PRILCFG>(0b10u)       // high priority
  );
}

} // namespace dma
} // namespace f4
} // namespace apm32
