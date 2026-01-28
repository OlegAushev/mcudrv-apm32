#pragma once

#include <apm32/f4/adc/adc_channels.hpp>
#include <apm32/f4/adc/adc_init.hpp>
#include <apm32/f4/adc/adc_instances.hpp>
#include <apm32/f4/adc/adc_traits.hpp>
#include <apm32/f4/adc/adc_types.hpp>
#include <apm32/f4/adc/adc_utils.hpp>

#include <optional>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

namespace detail {

struct basic_adc_config {
  unsigned injected_count;
  unsigned regular_count;
  bool dma_enabled;
  std::optional<inj_trigger> injected_trigger;
  std::optional<reg_trigger> regular_trigger;
  bool eoc_on_each_conversion;
  bool auto_injected_conversion;
};

void init_basic_adc(registers& regs, detail::basic_adc_config const& conf);

} // namespace detail

template<traits::basic_traits Traits, typename... Channels>
class basic_adc {
public:
  using adc_instance = Traits::adc_instance;
  using dma_stream = Traits::dma_stream;
  using dma_channel = Traits::dma_channel;
  using dma_stream_type = Traits::stream_type;
  static constexpr unsigned injected_count = Traits::injected_count;
  static constexpr unsigned regular_count = Traits::regular_count;
  static constexpr bool dma_enabled = Traits::dma_enabled;
  static constexpr nvic::irq_priority dma_irq_priority =
      Traits::dma_irq_priority;
  static constexpr std::optional<inj_trigger> injected_trigger =
      Traits::injected_trigger;
  static constexpr std::optional<reg_trigger> regular_trigger =
      Traits::regular_trigger;
  static constexpr bool eoc_on_each = Traits::eoc_on_each;
  static constexpr bool auto_injconv = Traits::auto_injconv;
private:
  static inline registers& regs_ = adc_instance::regs;
  dma_stream_type dma_stream_;
  std::array<std::optional<gpio::analog_pin>, sizeof...(Channels)> pins_;
public:
  basic_adc()
    requires(!dma_enabled) {
    adc_instance::enable_clock();
    detail::init_basic_adc(regs_, get_config());
    init_channels();
  }

  basic_adc()
    requires(dma_enabled)
      : dma_stream_(
            dma::v2::peripheral_to_memory_stream_config{
                .irq_priority = dma_irq_priority
            },
            &regs_.REGDATA
        ) {
    adc_instance::enable_clock();
    detail::init_basic_adc(regs_, get_config());
    init_channels();
  }

public:
  void enable() {
    if constexpr (dma_enabled) {
      dma_stream_.enable();
    }
    nvic::set_irq_priority(adc_instance::irqn, common_irq_priority);
    nvic::enable_irq(adc_instance::irqn);
  }

  void start_injected()
    requires(injected_count > 0 && !injected_trigger && !auto_injconv) {
    regs_.CTRL2_B.INJSWSC = 1;
  }

  void start_regular()
    requires(regular_count > 0 && !regular_trigger) {
    regs_.CTRL2_B.REGSWSC = 1;
  }

  template<unsigned Channel>
    requires(1 <= Channel && Channel <= injected_count)
  [[nodiscard]] uint32_t injected_result() const {
    return *reinterpret_cast<uint32_t volatile*>(
        adc_instance::reg_addr::jdrx[Channel - 1]
    );
  }

  template<unsigned Channel>
    requires(1 <= Channel && Channel <= injected_count)
  [[nodiscard]] uint32_t const volatile* injected_storage() const {
    return reinterpret_cast<uint32_t const volatile*>(
        adc_instance::reg_addr::jdrx[Channel - 1]
    );
  }

  template<unsigned Rank>
    requires(1 <= Rank && Rank <= regular_count && dma_enabled)
  [[nodiscard]] uint32_t const volatile* regular_storage() const {
    return &dma_stream_.data().data[Rank - 1];
  }
private:
  void init_channels() {
    [[maybe_unused]] size_t i = 0;
    (
        [&] {
          if (auto conf = Channels::init(regs_)) {
            pins_[i].emplace(*conf);
          }
          ++i;
        }(),
        ...
    );
  }

  detail::basic_adc_config get_config() const {
    return detail::basic_adc_config{
        .injected_count = injected_count,
        .regular_count = regular_count,
        .dma_enabled = dma_enabled,
        .injected_trigger = injected_trigger,
        .regular_trigger = regular_trigger,
        .eoc_on_each_conversion = eoc_on_each,
        .auto_injected_conversion = auto_injconv,
    };
  }
};

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
