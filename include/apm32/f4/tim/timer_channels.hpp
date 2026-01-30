#pragma once

#include <apm32/f4/tim/timer_types.hpp>

#include <emb/meta.hpp>

#include <tuple>

namespace apm32 {
namespace f4 {
namespace tim {

enum class channel_idx : unsigned {
  ch1,
  ch2,
  ch3,
  ch4
};

struct channel1 {
  static constexpr auto idx = channel_idx::ch1;
};

struct channel2 {
  static constexpr auto idx = channel_idx::ch2;
};

struct channel3 {
  static constexpr auto idx = channel_idx::ch3;
};

struct channel4 {
  static constexpr auto idx = channel_idx::ch4;
};

template<typename T>
struct is_timer_channel_instance
    : std::bool_constant<
          emb::same_as_any<T, channel1, channel2, channel3, channel4>> {};

template<typename T>
concept timer_channel_instance = is_timer_channel_instance<T>::value;

template<size_t I>
  requires(I < 4)
using channel_at = std::tuple_element_t<
    I,
    std::tuple<channel1, channel2, channel3, channel4>>;

} // namespace tim
} // namespace f4
} // namespace apm32
