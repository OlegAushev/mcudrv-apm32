#pragma once

#include <apm32/f4/adc/adc.hpp>

#include <array>
#include <cstddef>
#include <optional>

namespace apm32::f4::adc::detail {

// Shared low-level configuration for a sequenced ADC (regular and/or injected
// scan, optional triggers, optional DMA). Used by both multi_channel_adc and
// streaming_adc.
struct sequence_config {
  unsigned injected_count;
  unsigned regular_count;
  bool dma_enabled;
  std::optional<inj_trigger> injected_trigger;
  std::optional<reg_trigger> regular_trigger;
  bool eoc_on_each_conversion;
  bool auto_injected_conversion;
};

void init_sequence(registers& reg, sequence_config const& conf);

// Cross-channel check: the ranks of all channels of the given kind must
// cover positions 1..count exactly — every position assigned once, no
// gaps, duplicates, or out-of-range ranks.
template<bool Injected, some_adc_channel... Channels>
consteval bool ranks_cover_exactly(unsigned count) {
  std::array<unsigned, 17> tally{}; // tally[rank], rank in 1..16
  (
      [&] {
        if constexpr (Channels::injected == Injected) {
          for (auto rank : Channels::ranks) {
            ++tally[rank];
          }
        }
      }(),
      ...
  );
  for (unsigned rank = 1; rank <= 16; ++rank) {
    unsigned const expected = (rank <= count) ? 1u : 0u;
    if (tally[rank] != expected) {
      return false;
    }
  }
  return true;
}

} // namespace apm32::f4::adc::detail
