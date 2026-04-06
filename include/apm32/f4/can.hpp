#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>
#include <apm32/f4/nvic.hpp>

#include <emb/can.hpp>
#include <emb/mmio.hpp>
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

using peripheral_registers = CAN_TypeDef;

inline constexpr size_t peripheral_count = 2;

inline std::array<peripheral_registers*, peripheral_count> const peripherals = {
    CAN1,
    CAN2
};

enum class peripheral_id : uint32_t { can1, can2 };

struct rx_pin_config {
  gpio::port port;
  gpio::pin pin;
  uint32_t altfunc;
};

struct tx_pin_config {
  gpio::port port;
  gpio::pin pin;
  uint32_t altfunc;
};

struct config {
  uint32_t bittim;  // raw BITTIM register value
  uint32_t mctrl;   // raw MCTRL register value (mode bits)
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

inline constexpr std::array<uint32_t, peripheral_count> clock_bits = {
    RCM_APB1CLKEN_CAN1EN,
    RCM_APB1CLKEN_CAN2EN,
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

struct filter_config {
  uint32_t filter_number;
  uint32_t filter_id_high;
  uint32_t filter_id_low;
  uint32_t filter_mask_id_high;
  uint32_t filter_mask_id_low;
  rx_fifo fifo;
  bool scale_32bit;  // true = single 32-bit, false = dual 16-bit
  bool mode_id_list; // true = id list, false = id/mask
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

  rxmessage_attr register_rxmessage(filter_config filter);

  peripheral_id id() const {
    return id_;
  }

  peripheral_registers* regs() {
    return regs_;
  }

  void start();
  void stop();

  bool mailbox_full() const {
    if (!emb::mmio::test_any(regs_->TXSTS,
            CAN_TXSTS_TXMEFLG0 | CAN_TXSTS_TXMEFLG1 | CAN_TXSTS_TXMEFLG2)) {
      return true;
    }
    return false;
  }

  uint32_t rxfifo_level(rx_fifo fifo) const {
    switch (fifo) {
    case rx_fifo::fifo0:
      return emb::mmio::read(regs_->RXF0, CAN_RXF0_FMNUM0);
    case rx_fifo::fifo1:
      return emb::mmio::read(regs_->RXF1, CAN_RXF1_FMNUM1);
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
