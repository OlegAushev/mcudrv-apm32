#include <apm32/f4/can.hpp>

#include <emb/chrono.hpp>

namespace apm32 {
namespace f4 {
namespace can {

detail::rx_pin::rx_pin(rx_pin_config const& conf)
    : gpio::alternate_pin(
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::fast,
           .altfunc = conf.altfunc}
      ) {}

detail::tx_pin::tx_pin(tx_pin_config const& conf)
    : gpio::alternate_pin(
          {.port = conf.port,
           .pin = conf.pin,
           .pull = gpio::pull::none,
           .output_type = gpio::output_type::pushpull,
           .speed = gpio::speed::fast,
           .altfunc = conf.altfunc}
      ) {}

peripheral::peripheral(
    peripheral_id id,
    rx_pin_config const& rx_pinconf,
    tx_pin_config const& tx_pinconf,
    config conf
)
    : peripheral_type(id),
      id_(id),
      regs_(peripherals[std::to_underlying(id)]),
      rx_pin_(rx_pinconf),
      tx_pin_(tx_pinconf) {
  enable_clock(id_);
  CAN_Config(regs_, &conf.hal_config);
}

rxmessage_attr peripheral::register_rxmessage(CAN_FilterConfig_T filter) {
  core::ensure(filter_count_ < max_fitler_count_);

  rxmessage_attr attr{};

  if (id_ == peripheral_id::can1) {
    filter.filterNumber = filter_count_++;
  } else {
    filter.filterNumber = 14 + filter_count_++;
  }
  filter.filterActivation = ENABLE;
  //filter.SlaveStartFilterBank = 14;

  attr.filter_idx = filter.filterNumber;
  attr.fifo = static_cast<rx_fifo>(filter.filterFIFO);

  CAN_ConfigFilter(&filter);

  return attr;
}

void peripheral::start() {
  regs_->MCTRL_B.INITREQ = 0;
  emb::chrono::watchdog start_wd(std::chrono::milliseconds(2));
  while (regs_->MSTS_B.INITFLG == 1) {
    if (!start_wd.good()) {
      while (true) {
        // TODO
      }
    }
  }
}

void peripheral::stop() {
  regs_->MCTRL_B.INITREQ = 1;
  emb::chrono::watchdog stop_wd(std::chrono::milliseconds(2));
  while (regs_->MSTS_B.INITFLG == 0) {
    if (!stop_wd.good()) {
      while (true) {
        // TODO
      }
    }
  }
}

exec_status peripheral::put_frame(can_frame const& frame) {
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
  write_reg(
      regs_->sTxMailBox[mailboxid].TXMDL,
      (uint32_t(frame.payload[0]) << 0) | (uint32_t(frame.payload[1]) << 8) |
          (uint32_t(frame.payload[2]) << 16) |
          (uint32_t(frame.payload[3]) << 24)
  );
  write_reg(
      regs_->sTxMailBox[mailboxid].TXMDH,
      (uint32_t(frame.payload[4]) << 0) | (uint32_t(frame.payload[5]) << 8) |
          (uint32_t(frame.payload[6]) << 16) |
          (uint32_t(frame.payload[7]) << 24)
  );

  // request transmission
  regs_->sTxMailBox[mailboxid].TXMID_B.TXMREQ = 1;

  return exec_status::ok;
}

std::optional<rxmessage_attr>
peripheral::get_frame(can_frame& frame, rx_fifo fifo) const {
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

  rxmessage_attr attr{};
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
  case rx_fifo::fifo0:
    regs_->RXF0_B.RFOM0 = 1;
    break;
  case rx_fifo::fifo1:
    regs_->RXF1_B.RFOM1 = 1;
    break;
  }

  return {attr};
}

void peripheral::configure_interrupts(uint32_t interrupt_bitset) {
  set_bit(regs_->INTEN, interrupt_bitset);
}

void peripheral::set_interrupts_priority(
    nvic::irq_priority rx0_priority,
    nvic::irq_priority rx1_priority,
    nvic::irq_priority tx_priority
) {
  set_irq_priority(
      detail::rx0_irq_numbers[std::to_underlying(id_)],
      rx0_priority
  );
  set_irq_priority(
      detail::rx1_irq_numbers[std::to_underlying(id_)],
      rx1_priority
  );
  set_irq_priority(
      detail::tx_irq_numbers[std::to_underlying(id_)],
      tx_priority
  );
}

void peripheral::enable_interrupts() {
  nvic::enable_irq(detail::rx0_irq_numbers[std::to_underlying(id_)]);
  nvic::enable_irq(detail::rx1_irq_numbers[std::to_underlying(id_)]);
  nvic::enable_irq(detail::tx_irq_numbers[std::to_underlying(id_)]);
}

void peripheral::disable_interrupts() {
  nvic::disable_irq(detail::rx0_irq_numbers[std::to_underlying(id_)]);
  nvic::disable_irq(detail::rx1_irq_numbers[std::to_underlying(id_)]);
  nvic::disable_irq(detail::tx_irq_numbers[std::to_underlying(id_)]);
}

void peripheral::enable_clock(peripheral_id id) {
  auto can_idx{std::to_underlying(id)};
  if (is_clock_enabled_[can_idx]) {
    return;
  }

  detail::enable_clock[can_idx]();
  is_clock_enabled_[can_idx] = true;
}

} // namespace can
} // namespace f4
} // namespace apm32

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
