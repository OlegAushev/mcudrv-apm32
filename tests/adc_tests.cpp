#include <apm32/f4/adc.hpp>

using namespace apm32::f4;
using namespace apm32::f4::adc::v2;

namespace {

static_assert(detail::to_sdk(2) == ADC_PRESCALER_DIV2);
static_assert(detail::to_sdk(4) == ADC_PRESCALER_DIV4);
static_assert(detail::to_sdk(6) == ADC_PRESCALER_DIV6);
static_assert(detail::to_sdk(8) == ADC_PRESCALER_DIV8);

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
  using dma_stream = dma::v2::dma2_stream0;
  using dma_channel = dma::v2::channel0;
  using stream_type = dma::v2::peripheral_to_memory_stream<
      dma_stream,
      dma_channel,
      dma::v2::memory_buffer<uint32_t, regular_count>>;
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

static_assert(traits::basic_traits<adc_traits_1>);
static_assert(traits::basic_traits<adc_traits_2>);

using testing_extinj_channel =
    injected_channel<adc123_in0, sampletime::cycles_3, 1, 2, 3, 4>;
using testing_extreg_channel =
    regular_channel<adc123_in1, sampletime::cycles_3, 1>;
using testing_intinj_channel =
    injected_channel<adc1_in16, sampletime::cycles_144, 2>;

[[maybe_unused]] void test() {
  basic_adc<
      adc_traits_1,
      testing_extinj_channel,
      testing_extreg_channel,
      testing_intinj_channel>
      adc_1;
  basic_adc<adc_traits_2> adc_2;

  adc_1.enable();
  adc_2.enable();

  // adc_1.start_injected();
  adc_2.start_injected();

  // adc_1.start_regular();
  adc_2.start_regular();

  [[maybe_unused]] auto res1 = adc_1.injected_result<1>();
  // [[maybe_unused]] auto res2 = adc_2.injected_result<2>();
}

} // namespace
