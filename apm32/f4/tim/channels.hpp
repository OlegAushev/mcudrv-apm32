#pragma once

#include <apm32/f4/tim.hpp>

#include <emb/meta.hpp>
#include <emb/mmio.hpp>

#include <cstddef>
#include <cstdint>
#include <tuple>

namespace apm32::f4::tim {

enum class channel_idx : unsigned { ch1, ch2, ch3, ch4 };

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
concept some_timer_channel_instance = is_timer_channel_instance<T>::value;

template<std::size_t I>
  requires(I < 4)
using channel_at =
    std::tuple_element_t<I, std::tuple<channel1, channel2, channel3, channel4>>;

enum class capture_filter : std::uint32_t {
  disabled,
  div1_n2,
  div1_n4,
  div1_n8,
  div2_n6,
  div2_n8,
  div4_n6,
  div4_n8,
  div8_n6,
  div8_n8,
  div16_n5,
  div16_n6,
  div16_n8,
  div32_n5,
  div32_n6,
  div32_n8,
};

template<some_timer_instance Tim, some_timer_channel_instance Ch>
bool capture_compare_flag()
{
  if constexpr (std::same_as<Ch, channel1>) {
    return emb::mmio::test<TMR_STS_CC1IFLG>(Tim::reg.STS);
  }
  else if constexpr (std::same_as<Ch, channel2>) {
    return emb::mmio::test<TMR_STS_CC2IFLG>(Tim::reg.STS);
  }
  else if constexpr (std::same_as<Ch, channel3>) {
    return emb::mmio::test<TMR_STS_CC3IFLG>(Tim::reg.STS);
  }
  else if constexpr (std::same_as<Ch, channel4>) {
    return emb::mmio::test<TMR_STS_CC4IFLG>(Tim::reg.STS);
  }
  else {
    std::unreachable();
  }
}

template<some_timer_instance Tim, some_timer_channel_instance Ch>
void acknowledge_capture_compare()
{
  if constexpr (std::same_as<Ch, channel1>) {
    emb::mmio::clear_w0<TMR_STS_CC1IFLG>(Tim::reg.STS);
  }
  else if constexpr (std::same_as<Ch, channel2>) {
    emb::mmio::clear_w0<TMR_STS_CC2IFLG>(Tim::reg.STS);
  }
  else if constexpr (std::same_as<Ch, channel3>) {
    emb::mmio::clear_w0<TMR_STS_CC3IFLG>(Tim::reg.STS);
  }
  else if constexpr (std::same_as<Ch, channel4>) {
    emb::mmio::clear_w0<TMR_STS_CC4IFLG>(Tim::reg.STS);
  }
  else {
    std::unreachable();
  }
}

} // namespace apm32::f4::tim
