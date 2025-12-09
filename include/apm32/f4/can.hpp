#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <apm32f4xx_can.h>

#include <emb/can.hpp>
#include <emb/queue.hpp>

extern "C" {
void CAN1_RX0_IRQHandler();
void CAN1_RX1_IRQHandler();
void CAN1_TX_IRQHandler();
void CAN2_RX0_IRQHandler();
void CAN2_RX1_IRQHandler();
void CAN2_TX_IRQHandler();
}

namespace apm32 {
namespace f4 {
namespace can {

using peripheral_registers = CAN_T;

inline constexpr size_t peripheral_count = 2;

inline std::array<peripheral_registers*, peripheral_count> const peripherals = {
    CAN1,
    CAN2
};

enum class peripheral_id : uint32_t { can1, can2 };

struct rx_pin_config {
  gpio::port port;
  gpio::pin pin;
  GPIO_AF_T altfunc;
};

struct tx_pin_config {
  gpio::port port;
  gpio::pin pin;
  GPIO_AF_T altfunc;
};

struct config {
  CAN_Config_T hal_config;
};

namespace detail {

class rx_pin : public gpio::alternate_pin {
public:
  rx_pin(rx_pin_config const& conf);
};

class tx_pin : public gpio::alternate_pin {
public:
  tx_pin(tx_pin_config const& conf);
};

inline std::array<void (*)(void), peripheral_count> enable_clock = {
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN1); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN2); }
};

inline constexpr std::array<nvic::irq_number, peripheral_count>
    rx0_irq_numbers = {CAN1_RX0_IRQn, CAN2_RX0_IRQn};
inline constexpr std::array<nvic::irq_number, peripheral_count>
    rx1_irq_numbers = {CAN1_RX1_IRQn, CAN2_RX1_IRQn};
inline constexpr std::array<nvic::irq_number, peripheral_count> tx_irq_numbers =
    {CAN1_TX_IRQn, CAN2_TX_IRQn};

} // namespace detail

enum class rx_fifo { fifo0, fifo1 };

struct rxmessage_attr {
  rx_fifo fifo;
  uint32_t filter_idx = 0xBAAAAAAD; // FIXME
  bool operator==(rxmessage_attr const&) const = default;
};

class peripheral
    : public core::peripheral<peripheral, peripheral_id, peripheral_count> {
public:
  using peripheral_type =
      core::peripheral<peripheral, peripheral_id, peripheral_count>;
private:
  peripheral_id const id_;
  peripheral_registers* const regs_;
  detail::rx_pin rx_pin_;
  detail::tx_pin tx_pin_;

  uint8_t filter_count_ = 0;
#ifdef CAN2
  static const uint8_t max_fitler_count_ = 28;
#else
  static const uint8_t max_fitler_count_ = 14;
#endif

  emb::queue<can_frame, 32> txqueue_;
public:
  peripheral(
      peripheral_id id,
      rx_pin_config const& rx_pinconf,
      tx_pin_config const& tx_pinconf,
      config conf
  );

  rxmessage_attr register_rxmessage(CAN_FilterConfig_T filter);

  peripheral_id id() const {
    return id_;
  }

  peripheral_registers* regs() {
    return regs_;
  }

  void start();
  void stop();

  bool mailbox_full() const {
    if ((regs_->TXSTS_B.TXMEFLG0 + regs_->TXSTS_B.TXMEFLG1 +
         regs_->TXSTS_B.TXMEFLG2) == 0) {
      return true;
    }
    return false;
  }

  uint32_t rxfifo_level(rx_fifo fifo) const {
    switch (fifo) {
    case rx_fifo::fifo0:
      return regs_->RXF0_B.FMNUM0;
    case rx_fifo::fifo1:
      return regs_->RXF1_B.FMNUM1;
    }
    return 0;
  }

  exec_status put_frame(can_frame const& frame);

  std::optional<rxmessage_attr> get_frame(can_frame& frame, rx_fifo fifo) const;
public:
  void configure_interrupts(uint32_t interrupt_bitset);
  void set_interrupts_priority(
      nvic::irq_priority rx0_priority,
      nvic::irq_priority rx1_priority,
      nvic::irq_priority tx_priority
  );
  void enable_interrupts();
  void disable_interrupts();

  void on_mailbox_empty() {
    while (!mailbox_full()) {
      if (txqueue_.empty()) {
        return;
      }
      put_frame(txqueue_.front());
      txqueue_.pop();
    }
  }
private:
  static inline std::array<bool, peripheral_count> is_clock_enabled_{};
  static void enable_clock(peripheral_id id);
};

} // namespace can
} // namespace f4
} // namespace apm32
