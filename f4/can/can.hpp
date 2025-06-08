#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_can.h>

#include <mcudrv-apm32/f4/gpio/gpio.hpp>
#include <mcudrv-apm32/f4/system/system.hpp>

#include <emblib/can.hpp>
#include <emblib/singleton.hpp>

#include <emblib/core.hpp>
#include <emblib/queue.hpp>

extern "C" {
void CAN1_RX0_IRQHandler();
void CAN1_RX1_IRQHandler();
void CAN1_TX_IRQHandler();
void CAN2_RX0_IRQHandler();
void CAN2_RX1_IRQHandler();
void CAN2_TX_IRQHandler();
}

namespace mcu {
namespace apm32 {
namespace can {

using Regs = CAN_T;

constexpr size_t periph_num{2};

enum class Peripheral : size_t {
  can1,
  can2
};

inline std::array<Regs*, periph_num> const regs{CAN1, CAN2};

inline Peripheral get_peripheral(Regs const* reg) {
  return static_cast<Peripheral>(
      std::distance(regs.begin(), std::find(regs.begin(), regs.end(), reg)));
}

struct RxPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct TxPinConfig {
  gpio::Port port;
  gpio::Pin pin;
  GPIO_AF_T altfunc;
};

struct Config {
  CAN_Config_T hal_config;
};

namespace internal {

class RxPin : public gpio::AlternatePin {
public:
  RxPin(RxPinConfig const& conf);
};

class TxPin : public gpio::AlternatePin {
public:
  TxPin(TxPinConfig const& conf);
};

inline constexpr std::array<IRQn_Type, periph_num> can_rx0_irqn = {
    CAN1_RX0_IRQn, CAN2_RX0_IRQn};
inline constexpr std::array<IRQn_Type, periph_num> can_rx1_irqn = {
    CAN1_RX1_IRQn, CAN2_RX1_IRQn};
inline constexpr std::array<IRQn_Type, periph_num> can_tx_irqn = {CAN1_TX_IRQn,
                                                                  CAN2_TX_IRQn};
} // namespace internal

enum class RxFifo {
  fifo0,
  fifo1
};

struct RxMessageAttribute {
  RxFifo fifo;
  uint32_t filter_idx{0xBAAAAAAD}; // FIXME
  bool operator==(RxMessageAttribute const&) const = default;
};

class Module : public emb::singleton_array<Module, periph_num>,
               private emb::noncopyable {
private:
  Peripheral const peripheral_;
  Regs* const regs_;
  internal::RxPin rx_pin_;
  internal::TxPin tx_pin_;

  uint8_t filter_count_{0};
#ifdef CAN2
  static const uint8_t max_fitler_count_{28};
#else
  static const uint8_t max_fitler_count_{14};
#endif

  emb::queue<can_frame, 32> txqueue_;
public:
  Module(Peripheral peripheral,
         RxPinConfig const& rx_pin_conf,
         TxPinConfig const& tx_pin_conf,
         Config const& conf);
  RxMessageAttribute register_rxmessage(CAN_FilterConfig_T const& filter);

  Peripheral peripheral() const { return peripheral_; }

  Regs* regs() { return regs_; }

  static Module* instance(Peripheral peripheral) {
    return emb::singleton_array<Module, periph_num>::instance(
        std::to_underlying(peripheral));
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

  uint32_t rxfifo_level(RxFifo fifo) const {
    switch (fifo) {
    case RxFifo::fifo0:
      return regs_->RXF0_B.FMNUM0;
    case RxFifo::fifo1:
      return regs_->RXF1_B.FMNUM1;
    }
    return 0;
  }

  exec_status put_frame(can_frame const& frame);
  std::optional<RxMessageAttribute> get_frame(can_frame& frame,
                                              RxFifo fifo) const;
public:
  void init_interrupts(uint32_t interrupt_bitset);
  void set_interrupt_priority(IrqPriority rx0_priority,
                              IrqPriority rx1_priority,
                              IrqPriority tx_priority);
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
  static inline std::array<bool, periph_num> clk_enabled_{};
  static void enable_clk(Peripheral peripheral);
  static inline std::array<void (*)(void), periph_num> enable_clk_ = {
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN1); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN2); }};
};

} // namespace can
} // namespace apm32
} // namespace mcu

#endif
#endif
