#pragma once

#include <emb/meta.hpp>

namespace apm32 {
namespace f4 {
namespace tim {

struct channel1 {
  static constexpr unsigned idx = 0;
};

struct channel2 {
  static constexpr unsigned idx = 1;
};

struct channel3 {
  static constexpr unsigned idx = 2;
};

struct channel4 {
  static constexpr unsigned idx = 3;
};

template<typename T>
struct is_timer_channel_instance
    : std::bool_constant<
          emb::same_as_any<T, channel1, channel2, channel3, channel4>> {};

template<typename T>
concept timer_channel_instance = is_timer_channel_instance<T>::value;

} // namespace tim
} // namespace f4
} // namespace apm32
