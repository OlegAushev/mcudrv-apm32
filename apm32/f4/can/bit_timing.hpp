#pragma once

#include <apm32/f4/can.hpp>

#include <emb/assert.hpp>

#include <algorithm>
#include <cstdint>
#include <optional>

namespace apm32::f4::can {

struct bit_timing {
  static constexpr std::uint32_t prescaler_max = 1024;
  static constexpr std::uint32_t sync_jump_width_max = 4;
  static constexpr std::uint32_t time_segment1_max = 16;
  static constexpr std::uint32_t time_segment2_max = 8;

  std::uint16_t prescaler;
  std::uint8_t sync_jump_width;
  std::uint8_t time_segment1;
  std::uint8_t time_segment2;

  bool operator==(bit_timing const&) const = default;

  constexpr std::uint32_t reg_bits() const
  {
    emb::ensure(prescaler >= 1 && prescaler <= prescaler_max);
    emb::ensure(sync_jump_width >= 1 && sync_jump_width <= sync_jump_width_max);
    emb::ensure(time_segment1 >= 1 && time_segment1 <= time_segment1_max);
    emb::ensure(time_segment2 >= 1 && time_segment2 <= time_segment2_max);

    return ((prescaler - 1u) << CAN_BITTIM_BRPSC_Pos)
         | ((sync_jump_width - 1u) << CAN_BITTIM_RSYNJW_Pos)
         | ((time_segment1 - 1u) << CAN_BITTIM_TIMSEG1_Pos)
         | ((time_segment2 - 1u) << CAN_BITTIM_TIMSEG2_Pos);
  }
};

namespace detail {

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
