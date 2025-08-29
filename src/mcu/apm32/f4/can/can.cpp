#ifdef APM32F4XX

#include <mcu/apm32/f4/can.hpp>
#include <mcu/apm32/f4/chrono.hpp>

#include <emb/chrono.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace can {

internal::RxPin::RxPin(RxPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::pushpull,
                          .speed = gpio::Speed::fast,
                          .altfunc = conf.altfunc}} {}

internal::TxPin::TxPin(TxPinConfig const& conf)
    : gpio::AlternatePin{{.port = conf.port,
                          .pin = conf.pin,
                          .pull = gpio::Pull::none,
                          .output_type = gpio::OutputType::pushpull,
                          .speed = gpio::Speed::fast,
                          .altfunc = conf.altfunc}} {}

Module::Module(Peripheral peripheral,
               RxPinConfig const& rx_pinconf,
               TxPinConfig const& tx_pinconf,
               Config const& conf)
    : emb::singleton_array<Module, periph_num>(this,
                                               std::to_underlying(peripheral)),
      peripheral_(peripheral),
      regs_{can::regs[static_cast<size_t>(peripheral_)]},
      rx_pin_{rx_pinconf},
      tx_pin_{tx_pinconf} {
  enable_clk(peripheral);
  CAN_Config(regs_, const_cast<CAN_Config_T*>(&conf.hal_config));
}

RxMessageAttribute
Module::register_rxmessage(CAN_FilterConfig_T const& filter) {
  auto filt{filter};
  RxMessageAttribute attr{};

  if (filter_count_ >= max_fitler_count_) {
    fatal_error();
  }

  if (peripheral_ == Peripheral::can1) {
    filt.filterNumber = filter_count_++;
  } else {
    filt.filterNumber = 14 + filter_count_++;
  }
  filt.filterActivation = ENABLE;
  //filter.SlaveStartFilterBank = 14;

  attr.filter_idx = filt.filterNumber;
  attr.fifo = RxFifo(filt.filterFIFO);

  CAN_ConfigFilter(&filt);

  return attr;
}

void Module::start() {
  regs_->MCTRL_B.INITREQ = 0;
  emb::chrono::watchdog start_wd(std::chrono::milliseconds(2));
  while (regs_->MSTS_B.INITFLG == 1) {
    if (!start_wd.good()) {
      fatal_error();
    }
  }
}

void Module::stop() {
  regs_->MCTRL_B.INITREQ = 1;
  emb::chrono::watchdog stop_wd(std::chrono::milliseconds(2));
  while (regs_->MSTS_B.INITFLG == 0) {
    if (!stop_wd.good()) {
      fatal_error();
    }
  }
}

exec_status Module::put_frame(can_frame const& frame) {
  if (mailbox_full()) {
    if (txqueue_.full()) {
      return exec_status::overflow;
    }
    txqueue_.push(frame);
    return exec_status::busy;
  }

  uint32_t mailboxid = regs_->TXSTS_B.EMNUM;
  if (mailboxid > 2) {
    return exec_status::error;
  }

  // set up id
  if (frame.id <= 0x7FF) {
    write_reg(regs_->sTxMailBox[mailboxid].TXMID, (frame.id << 21));
  } else if (frame.id <= 0x1FFFFFFF) {
    write_reg(regs_->sTxMailBox[mailboxid].TXMID, (frame.id << 3));
    regs_->sTxMailBox[mailboxid].TXMID_B.IDTYPESEL = 1;
  } else {
    return exec_status::invalid_argument;
  }

  // set up dlc
  regs_->sTxMailBox[mailboxid].TXDLEN_B.DLCODE = frame.len & 0x0F;

  // set up data field
  write_reg(regs_->sTxMailBox[mailboxid].TXMDL,
            (uint32_t(frame.payload[0]) << 0) |
                (uint32_t(frame.payload[1]) << 8) |
                (uint32_t(frame.payload[2]) << 16) |
                (uint32_t(frame.payload[3]) << 24));
  write_reg(regs_->sTxMailBox[mailboxid].TXMDH,
            (uint32_t(frame.payload[4]) << 0) |
                (uint32_t(frame.payload[5]) << 8) |
                (uint32_t(frame.payload[6]) << 16) |
                (uint32_t(frame.payload[7]) << 24));

  // request transmission
  regs_->sTxMailBox[mailboxid].TXMID_B.TXMREQ = 1;

  return exec_status::ok;
}

std::optional<RxMessageAttribute> Module::get_frame(can_frame& frame,
                                                    RxFifo fifo) const {
  if (rxfifo_level(fifo) == 0) {
    return {};
  }

  auto fifo_idx = std::to_underlying(fifo);

  // get id, len, filter
  if (regs_->sRxMailBox[fifo_idx].RXMID_B.IDTYPESEL == 0) {
    frame.id = regs_->sRxMailBox[fifo_idx].RXMID >> 21;
  } else {
    frame.id = regs_->sRxMailBox[fifo_idx].RXMID >> 3;
  }

  frame.len = uint8_t(regs_->sRxMailBox[fifo_idx].RXDLEN_B.DLCODE);

  RxMessageAttribute attr{};
  attr.filter_idx = regs_->sRxMailBox[fifo_idx].RXDLEN_B.FMIDX;
  attr.fifo = fifo;

  // get data
  frame.payload[0] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE0);
  frame.payload[1] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE1);
  frame.payload[2] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE2);
  frame.payload[3] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE3);
  frame.payload[4] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE4);
  frame.payload[5] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE5);
  frame.payload[6] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE6);
  frame.payload[7] = uint8_t(regs_->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE7);

  // release fifo
  switch (fifo) {
  case RxFifo::fifo0:
    regs_->RXF0_B.RFOM0 = 1;
    break;
  case RxFifo::fifo1:
    regs_->RXF1_B.RFOM1 = 1;
    break;
  }

  return {attr};
}

void Module::init_interrupts(uint32_t interrupt_bitset) {
  set_bit(regs_->INTEN, interrupt_bitset);
}

void Module::set_interrupt_priority(IrqPriority rx0_priority,
                                    IrqPriority rx1_priority,
                                    IrqPriority tx_priority) {
  set_irq_priority(internal::can_rx0_irqn[std::to_underlying(peripheral_)],
                   rx0_priority);
  set_irq_priority(internal::can_rx1_irqn[std::to_underlying(peripheral_)],
                   rx1_priority);
  set_irq_priority(internal::can_tx_irqn[std::to_underlying(peripheral_)],
                   tx_priority);
}

void Module::enable_interrupts() {
  enable_irq(internal::can_rx0_irqn[std::to_underlying(peripheral_)]);
  enable_irq(internal::can_rx1_irqn[std::to_underlying(peripheral_)]);
  enable_irq(internal::can_tx_irqn[std::to_underlying(peripheral_)]);
}

void Module::disable_interrupts() {
  disable_irq(internal::can_rx0_irqn[std::to_underlying(peripheral_)]);
  disable_irq(internal::can_rx1_irqn[std::to_underlying(peripheral_)]);
  disable_irq(internal::can_tx_irqn[std::to_underlying(peripheral_)]);
}

void Module::enable_clk(Peripheral peripheral) {
  auto can_idx{std::to_underlying(peripheral)};
  if (clk_enabled_[can_idx]) {
    return;
  }

  enable_clk_[can_idx]();
  clk_enabled_[can_idx] = true;
}

} // namespace can
} // namespace f4
} // namespace apm32
} // namespace mcu

// extern "C" void CAN2_RX0_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
// }

// extern "C" void CAN1_RX1_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can1)->handle());
// }

// extern "C" void CAN2_RX1_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
// }

// extern "C" void CAN2_TX_IRQHandler() {
//     using namespace mcu::can;
//     HAL_CAN_IRQHandler(Module::instance(Peripheral::can2)->handle());
// }

#endif
