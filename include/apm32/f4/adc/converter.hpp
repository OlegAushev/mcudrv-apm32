#pragma once

#include <apm32/f4/adc/adc_types.hpp>
#include <apm32/f4/adc/adc_utils.hpp>

#include <apm32/f4/dma.hpp>

#include <optional>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

struct analog_to_digital_converter_config {
  bool eoc_on_each_conversion;
  external_trigger_edge trigger_edge;
  // std::optional<external_trigger> trigger;
  nvic::irq_priority dma_irq_priority;
};

namespace detail {
void common_init();
void init();
} // namespace detail

template<
    adc_module_instance Instance,
    size_t InjCount,
    size_t RegCount,
    dma::v2::dma_stream_instance DmaStream,
    dma::v2::dma_channel_instance DmaCh>
class analog_to_digital_converter {
public:
  using adc_instance = Instance;
  using dma_stream = DmaStream;
  using dma_channel = DmaCh;
  static constexpr size_t injected_channel_count = InjCount;
  static constexpr size_t regular_channel_count = RegCount;
private:
  static inline registers& regs = adc_instance::regs;

  dma::v2::peripheral_to_memory_stream<
      dma_stream,
      dma_channel,
      dma::v2::memory_buffer<uint32_t, regular_channel_count>>
      dma_stream_;
public:
  analog_to_digital_converter(analog_to_digital_converter_config const& conf)
    requires emb::in_type_list<
                 dma_stream,
                 typename adc_instance::dma_streams> &&
             std::same_as<dma_channel, typename adc_instance::dma_channel>
      : dma_stream_(
            dma::v2::peripheral_to_memory_stream_config{
                .irq_priority = conf.dma_irq_priority
            },
            &regs.REGDATA
        ) {
    adc_instance::enable_clock();
    detail::common_init();
  }
};
// std::array<float, 0> a;
// inline analog_to_digital_converter<
//     adc2,
//     2,
//     1,
//     dma::v2::dma2_stream2,
//     dma::v2::channel1>
//     foo;

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
