#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/meta.hpp>
#include <emb/mmio.hpp>

#include <cstddef>

namespace apm32 {
namespace f4 {
namespace can {

using registers = CAN_TypeDef;

inline constexpr size_t count = 2;

struct can1 {
  static inline registers& REG = *CAN1;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN1EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::can1;
};

struct can2 {
  static inline registers& REG = *CAN2;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN2EN);
  };

  static constexpr uint32_t gpio_altfunc = gpio::altfunc::can2;
};

template<typename T>
struct is_can_instance
    : std::bool_constant<emb::same_as_any<T, can1, can2>> {};

template<typename T>
concept some_can_instance = is_can_instance<T>::value;

} // namespace can
} // namespace f4
} // namespace apm32
