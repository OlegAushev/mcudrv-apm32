#pragma once

#include <emb/meta.hpp>

namespace apm32 {
namespace f4 {
namespace dma {
namespace v2 {

struct channel0 {
  static constexpr unsigned idx = 0;
};

struct channel1 {
  static constexpr unsigned idx = 1;
};

struct channel2 {
  static constexpr unsigned idx = 2;
};

struct channel3 {
  static constexpr unsigned idx = 3;
};

struct channel4 {
  static constexpr unsigned idx = 4;
};

struct channel5 {
  static constexpr unsigned idx = 5;
};

struct channel6 {
  static constexpr unsigned idx = 6;
};

struct channel7 {
  static constexpr unsigned idx = 7;
};

template<typename T>
struct is_dma_channel_instance : std::bool_constant<emb::same_as_any<
                                     T,
                                     channel0,
                                     channel1,
                                     channel2,
                                     channel3,
                                     channel4,
                                     channel5,
                                     channel6,
                                     channel7>> {};

template<typename T>
concept dma_channel_instance = is_dma_channel_instance<T>::value;

} // namespace v2
} // namespace dma
} // namespace f4
} // namespace apm32
