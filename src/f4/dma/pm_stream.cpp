#include <apm32/f4/dma/pm_stream.hpp>

namespace apm32 {
namespace f4 {
namespace dma {
namespace v2 {

void detail::init_pm_stream(stream_registers& regs, uint32_t ch) {
  DMA_Config_T dma_config{};
  dma_config.channel = static_cast<DMA_CHANNEL_T>(ch);
  dma_config.peripheralBaseAddr = 0; // initialized in ctor
  dma_config.memoryBaseAddr = 0;     // initialized in ctor
  dma_config.dir = DMA_DIR_PERIPHERALTOMEMORY;
  dma_config.bufferSize = 0; // initialized in ctor
  dma_config.peripheralInc = DMA_PERIPHERAL_INC_DISABLE;
  dma_config.memoryInc = DMA_MEMORY_INC_ENABLE;
  dma_config.peripheralDataSize = DMA_PERIPHERAL_DATA_SIZE_WORD;
  dma_config.memoryDataSize = DMA_MEMORY_DATA_SIZE_WORD;
  dma_config.loopMode = DMA_MODE_CIRCULAR;
  dma_config.priority = DMA_PRIORITY_HIGH;
  dma_config.fifoMode = DMA_FIFOMODE_DISABLE;
  dma_config.fifoThreshold = DMA_FIFOTHRESHOLD_HALFFULL;
  dma_config.memoryBurst = DMA_MEMORYBURST_SINGLE;
  dma_config.peripheralBurst = DMA_PERIPHERALBURST_SINGLE;

  DMA_Config(&regs, &dma_config);
}

} // namespace v2
} // namespace dma
} // namespace f4
} // namespace apm32
