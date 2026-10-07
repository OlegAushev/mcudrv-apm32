#include <apm32/f4/dma.hpp>

#include <apm32/f4/dma/pm_stream.hpp>

using namespace apm32::f4::dma;

namespace {

// the flags are laid out by stream, whatever request channel it serves:
// streams 0-3 clear through LIFCLR, 4-7 through HIFCLR, at the same offsets
static_assert(detail::interrupt_clear_mask<dma2_stream0>()
              == (DMA_LIFCLR_CTXCIFLG0
                  | DMA_LIFCLR_CTXEIFLG0
                  | DMA_LIFCLR_CDMEIFLG0));
static_assert(detail::interrupt_clear_mask<dma2_stream1>()
              == (DMA_LIFCLR_CTXCIFLG1
                  | DMA_LIFCLR_CTXEIFLG1
                  | DMA_LIFCLR_CDMEIFLG1));
static_assert(detail::interrupt_clear_mask<dma2_stream2>()
              == (DMA_LIFCLR_CTXCIFLG2
                  | DMA_LIFCLR_CTXEIFLG2
                  | DMA_LIFCLR_CDMEIFLG2));
static_assert(detail::interrupt_clear_mask<dma2_stream3>()
              == (DMA_LIFCLR_CTXCIFLG3
                  | DMA_LIFCLR_CTXEIFLG3
                  | DMA_LIFCLR_CDMEIFLG3));
static_assert(detail::interrupt_clear_mask<dma2_stream4>()
              == (DMA_HIFCLR_CTXCIFLG4
                  | DMA_HIFCLR_CTXEIFLG4
                  | DMA_HIFCLR_CDMEIFLG4));
static_assert(detail::interrupt_clear_mask<dma2_stream5>()
              == (DMA_HIFCLR_CTXCIFLG5
                  | DMA_HIFCLR_CTXEIFLG5
                  | DMA_HIFCLR_CDMEIFLG5));
static_assert(detail::interrupt_clear_mask<dma2_stream6>()
              == (DMA_HIFCLR_CTXCIFLG6
                  | DMA_HIFCLR_CTXEIFLG6
                  | DMA_HIFCLR_CDMEIFLG6));
static_assert(detail::interrupt_clear_mask<dma2_stream7>()
              == (DMA_HIFCLR_CTXCIFLG7
                  | DMA_HIFCLR_CTXEIFLG7
                  | DMA_HIFCLR_CDMEIFLG7));

} // namespace
