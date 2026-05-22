#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/delegate.hpp>
#include <emb/meta.hpp>
#include <emb/mmio.hpp>

#include <cstddef>
#include <cstdint>

namespace apm32::f4::can {

using registers = CAN_TypeDef;

using irq_handler = emb::delegate<void(void)>;

inline constexpr std::size_t count = 2;

struct can1 {
  static inline registers& reg = *CAN1;

  static constexpr nvic::irq_number rx0_irqn = CAN1_RX0_IRQn;
  static constexpr nvic::irq_number rx1_irqn = CAN1_RX1_IRQn;
  static constexpr nvic::irq_number tx_irqn = CAN1_TX_IRQn;
  static constexpr nvic::irq_number sce_irqn = CAN1_SCE_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN1EN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::can1;

  static inline irq_handler on_irq_rx0;
  static inline irq_handler on_irq_rx1;
  static inline irq_handler on_irq_tx;
  static inline irq_handler on_irq_sce;
};

struct can2 {
  static inline registers& reg = *CAN2;

  static constexpr nvic::irq_number rx0_irqn = CAN2_RX0_IRQn;
  static constexpr nvic::irq_number rx1_irqn = CAN2_RX1_IRQn;
  static constexpr nvic::irq_number tx_irqn = CAN2_TX_IRQn;
  static constexpr nvic::irq_number sce_irqn = CAN2_SCE_IRQn;

  template<typename T>
  static constexpr auto clock_frequency = core::apb1_frequency<T>;

  static constexpr auto enable_clock = []() {
    emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_CAN2EN);
  };

  static constexpr std::uint32_t gpio_altfunc = gpio::altfunc::can2;

  static inline irq_handler on_irq_rx0;
  static inline irq_handler on_irq_rx1;
  static inline irq_handler on_irq_tx;
  static inline irq_handler on_irq_sce;
};

template<typename T>
struct is_can_instance : std::bool_constant<emb::same_as_any<T, can1, can2>> {};

template<typename T>
concept some_can_instance = is_can_instance<T>::value;

} // namespace apm32::f4::can
