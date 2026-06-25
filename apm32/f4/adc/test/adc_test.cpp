#include <apm32/f4/adc/adc.hpp>
#include <apm32/f4/adc/multi_channel_adc.hpp>
#include <apm32/f4/adc/streaming_adc.hpp>

#include <cstdint>

using namespace apm32::f4;
using namespace apm32::f4::adc;

namespace {

static_assert(detail::prescaler_to_field(2) == 0);
static_assert(detail::prescaler_to_field(4) == 1);
static_assert(detail::prescaler_to_field(6) == 2);
static_assert(detail::prescaler_to_field(8) == 3);

static_assert(
    detail::calculate_prescaler(
        emb::units::hz_f32{84e6f},
        max_clock_frequency
    ) == 4
);

struct adc_traits_1 {
  using adc_instance = adc1;
  static constexpr unsigned injected_count = 2;
  static constexpr unsigned regular_count = 4;
  static constexpr bool dma_enabled = true;
  using dma_stream = dma::dma2_stream0;
  using dma_channel = dma::channel0;
  using stream_type = dma::peripheral_to_memory_stream<
      dma_stream,
      dma_channel,
      dma::owned_storage<dma::memory_buffer<std::uint32_t, regular_count>>>;
  static constexpr nvic::irq_priority dma_irq_priority{4};
  static constexpr auto injected_trigger = inj_trigger{
      .edge = trigger_edge::rising,
      .event = inj_trigger_event::tim1_trgo
  };
  static constexpr auto regular_trigger = reg_trigger{
      .edge = trigger_edge::falling,
      .event = reg_trigger_event::tim3_trgo
  };
  static constexpr bool eoc_on_each = true;
  static constexpr bool auto_injconv = true;
};

struct adc_traits_2 {
  using adc_instance = adc2;
  static constexpr unsigned injected_count = 1;
  static constexpr unsigned regular_count = 2;
  static constexpr bool dma_enabled = false;
  using dma_stream = void;
  using dma_channel = void;
  struct stream_type {};
  static constexpr auto injected_trigger = std::nullopt;
  static constexpr auto regular_trigger = std::nullopt;
  static constexpr bool eoc_on_each = false;
  static constexpr bool auto_injconv = false;
};

static_assert(some_multi_channel_adc_traits<adc_traits_1>);
static_assert(some_multi_channel_adc_traits<adc_traits_2>);

// adc_3: DMA-only double-buffered streaming, 3 regular channels, 4 frames per
// half-window (buffer size = 3 * 4 = 12 uint16_t).
struct stream_traits_3 {
  using adc_instance = adc3;
  static constexpr unsigned regular_count = 3;
  using dma_stream = dma::dma2_stream0;
  using dma_channel = dma::channel2;
  using stream_type = dma::peripheral_to_memory_stream<
      dma_stream,
      dma_channel,
      dma::owned_storage<
          dma::memory_double_buffer<std::uint16_t, regular_count * 4>>>;
  static constexpr nvic::irq_priority dma_irq_priority{4};
  static constexpr auto regular_trigger = reg_trigger{
      .edge = trigger_edge::rising,
      .event = reg_trigger_event::tim3_trgo
  };
};

static_assert(some_streaming_adc_traits<stream_traits_3>);

// adc_1: injected ranks cover {1, 2}, regular ranks cover {1, 2, 3, 4}
using adc1_inj1 = channel<adc123_in0, sampletime::cycles_3, injected_rank_sequence<1>>;
using adc1_inj2 = channel<adc1_in16, sampletime::cycles_144, injected_rank_sequence<2>>;
using adc1_reg1 = channel<adc123_in1, sampletime::cycles_3, regular_rank_sequence<1>>;
using adc1_reg2 = channel<adc123_in2, sampletime::cycles_3, regular_rank_sequence<2>>;
using adc1_reg3 = channel<adc123_in3, sampletime::cycles_3, regular_rank_sequence<3>>;
using adc1_reg4 = channel<adc123_in10, sampletime::cycles_3, regular_rank_sequence<4>>;

// adc_2: injected ranks cover {1}, regular ranks cover {1, 2}
using adc2_inj1 = channel<adc12_in4, sampletime::cycles_3, injected_rank_sequence<1>>;
using adc2_reg1 = channel<adc12_in5, sampletime::cycles_3, regular_rank_sequence<1>>;
using adc2_reg2 = channel<adc12_in6, sampletime::cycles_3, regular_rank_sequence<2>>;

// adc_3: regular ranks cover {1, 2, 3}
using adc3_reg1 = channel<adc3_in4, sampletime::cycles_3, regular_rank_sequence<1>>;
using adc3_reg2 = channel<adc3_in5, sampletime::cycles_3, regular_rank_sequence<2>>;
using adc3_reg3 = channel<adc3_in6, sampletime::cycles_3, regular_rank_sequence<3>>;

[[maybe_unused]] void test() {
  multi_channel_adc<
      adc_traits_1,
      adc1_inj1,
      adc1_inj2,
      adc1_reg1,
      adc1_reg2,
      adc1_reg3,
      adc1_reg4>
      adc_1;
  multi_channel_adc<adc_traits_2, adc2_inj1, adc2_reg1, adc2_reg2> adc_2;
  streaming_adc<stream_traits_3, adc3_reg1, adc3_reg2, adc3_reg3> adc_3;

  adc_1.enable();
  adc_2.enable();
  adc_3.enable();

  // adc_1 uses external triggers, so software start is constrained out
  adc_2.start_injected();
  adc_2.start_regular();

  [[maybe_unused]] auto res1 = adc_1.injected_result<1>();
  [[maybe_unused]] auto res2 = adc_2.injected_result<1>();

  // adc_3 is DMA-driven: ISR acks, consumer reads a completed window
  adc_3.on_dma_complete();
  if (auto window = adc_3.completed()) {
    [[maybe_unused]] auto first = (*window)[0];
    adc_3.consume();
  }
}

} // namespace
