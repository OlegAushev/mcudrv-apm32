#pragma once

#include <apm32/f4/can/low_layer/can_instances.hpp>
#include <apm32/f4/can/low_layer/can_types.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace apm32::f4::can {

struct filter_init_session {
  filter_init_session()
  {
    emb::mmio::set<CAN_FCTRL_FINITEN>(can1::reg.FCTRL);
  }

  ~filter_init_session()
  {
    emb::mmio::clear<CAN_FCTRL_FINITEN>(can1::reg.FCTRL);
  }
};

inline void setup_filter_bank(filter_scale scale,
                              filter_mode mode,
                              rx_fifo fifo,
                              std::size_t filter_idx,
                              std::uint32_t bank1,
                              std::uint32_t bank2)
{
  registers& reg = can1::reg;

  std::uint32_t const filter_bit = 1u << filter_idx;

  filter_init_session fg;

  // deactivate filter
  emb::mmio::runtime::clear(reg.FACT, filter_bit);

  emb::mmio::runtime::set_or_clear(reg.FSCFG,
                                   filter_bit,
                                   scale == filter_scale::_32bit);
  emb::mmio::runtime::set_or_clear(reg.FMCFG,
                                   filter_bit,
                                   mode == filter_mode::list);
  emb::mmio::runtime::set_or_clear(reg.FFASS, filter_bit, fifo == rx_fifo::_1);

  reg.sFilterRegister[filter_idx].FBANK1 = bank1;
  reg.sFilterRegister[filter_idx].FBANK2 = bank2;

  // activate filter
  emb::mmio::runtime::set(reg.FACT, filter_bit);
}

namespace detail {

constexpr std::uint32_t encode_32bit_id(emb::can::format_t fmt,
                                        emb::can::id_t id)
{
  if (fmt == emb::can::format_t::standard) {
    return (id & 0x7FFu) << 21;
  }
  constexpr std::uint32_t ide_bit = 1u << 2;
  return (id & 0x1FFFFFFFu) << 3 | ide_bit;
}

constexpr std::uint32_t encode_32bit_mask(emb::can::format_t fmt,
                                          emb::can::id_t mask)
{
  constexpr std::uint32_t rtr_bit = 1u << 1; // accept data frames only
  constexpr std::uint32_t ide_bit = 1u << 2;
  if (fmt == emb::can::format_t::standard) {
    return (mask & 0x7FFu) << 21 | ide_bit | rtr_bit;
  }
  return (mask & 0x1FFFFFFFu) << 3 | ide_bit | rtr_bit;
}

constexpr std::uint32_t encode_16bit_id(emb::can::id_t id)
{
  return (id & 0x7FFu) << 5;
}

constexpr std::uint32_t encode_16bit_mask(emb::can::id_t id)
{
  constexpr std::uint32_t rtr_bit = 1u << 4; // accept data frames only
  constexpr std::uint32_t ide_bit = 1u << 3;
  return (id & 0x7FFu) << 5 | ide_bit | rtr_bit;
}

inline constexpr float default_sample_point = 0.875f;
inline constexpr float sample_point_tolerance = 0.02f;
inline constexpr std::uint32_t default_sync_jump_width = 1;
inline constexpr std::uint32_t tq_per_bit_min = 8;
inline constexpr std::uint32_t tq_per_bit_max = 25;

void no_representable_bit_timing();
void sync_jump_width_out_of_range();

consteval std::optional<bit_timing> find_bit_timing(std::uint32_t clk_freq,
                                                    std::uint32_t bitrate,
                                                    float sample_point)
{
  for (std::uint32_t ntq = tq_per_bit_max; ntq >= tq_per_bit_min; --ntq) {
    std::uint32_t const clocks_per_bit = bitrate * ntq;
    if (clocks_per_bit == 0 || clk_freq % clocks_per_bit != 0) continue;

    std::uint32_t const prescaler = clk_freq / clocks_per_bit;
    if (prescaler == 0 || prescaler > bit_timing::prescaler_max) continue;

    std::uint32_t best_ts1 = 0;
    float best_error = 0.0f;
    for (std::uint32_t ts1 = 1;
         ts1 <= bit_timing::time_segment1_max && ts1 + 2 <= ntq;
         ++ts1) {
      std::uint32_t const ts2 = ntq - 1 - ts1;
      if (ts2 > bit_timing::time_segment2_max) continue;

      float const sp = float(1 + ts1) / float(ntq);
      float const error =
          sp > sample_point ? sp - sample_point : sample_point - sp;
      if (best_ts1 == 0 || error < best_error) {
        best_ts1 = ts1;
        best_error = error;
      }
    }

    if (best_ts1 == 0 || best_error > sample_point_tolerance) continue;

    std::uint32_t const ts2 = ntq - 1 - best_ts1;
    return bit_timing{.prescaler = std::uint16_t(prescaler),
                      .sync_jump_width = std::uint8_t(default_sync_jump_width),
                      .time_segment1 = std::uint8_t(best_ts1),
                      .time_segment2 = std::uint8_t(ts2)};
  }

  return {};
}

consteval bit_timing with_sync_jump_width(bit_timing timing,
                                          std::uint32_t sync_jump_width)
{
  std::uint32_t const width_max = std::min(bit_timing::sync_jump_width_max,
                                           std::uint32_t(timing.time_segment2));
  if (sync_jump_width < 1 || sync_jump_width > width_max) {
    sync_jump_width_out_of_range();
  }

  timing.sync_jump_width = std::uint8_t(sync_jump_width);
  return timing;
}

} // namespace detail

template<some_can_instance Instance>
consteval bit_timing calculate_bit_timing(
    std::uint32_t bitrate,
    std::uint32_t sync_jump_width = detail::default_sync_jump_width,
    float sample_point = detail::default_sample_point)
{
  auto const timing = detail::find_bit_timing(
      Instance::template clock_frequency<std::uint32_t>(),
      bitrate,
      sample_point);
  if (!timing) detail::no_representable_bit_timing();
  return detail::with_sync_jump_width(*timing, sync_jump_width);
}

} // namespace apm32::f4::can
