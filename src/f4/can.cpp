#include <apm32/f4/can.hpp>

#include <emb/chrono.hpp>
#include <emb/mmio.hpp>

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

  // enter init mode
  emb::mmio::set(regs_->MCTRL, CAN_MCTRL_INITREQ);
  emb::chrono::timeout init_t(std::chrono::milliseconds(2));
  while (!emb::mmio::test_any(regs_->MSTS, CAN_MSTS_INITFLG)) {
    if (init_t.expired()) {
      while (true) {}
    }
  }

  // configure
  regs_->MCTRL = (regs_->MCTRL & CAN_MCTRL_INITREQ) | conf.mctrl_reg();
  regs_->BITTIM = conf.bittim_reg();
}

rxmessage_attr peripheral::register_rxmessage(filter_config filter) {
  core::ensure(filter_count_ < max_fitler_count_);

  rxmessage_attr attr{};

  uint32_t filter_number;
  if (id_ == peripheral_id::can1) {
    filter_number = filter_count_++;
  } else {
    filter_number = 14 + filter_count_++;
  }

  attr.filter_idx = filter_number;
  attr.fifo = filter.fifo;

  uint32_t const filter_bit = 1u << filter_number;

  // enter filter init mode
  emb::mmio::set(regs_->FCTRL, CAN_FCTRL_FINITEN);

  // deactivate filter
  emb::mmio::clear(regs_->FACT, filter_bit);

  // set scale
  if (filter.scale_32bit) {
    emb::mmio::set(regs_->FSCFG, filter_bit);
  } else {
    emb::mmio::clear(regs_->FSCFG, filter_bit);
  }

  // set mode
  if (filter.mode_id_list) {
    emb::mmio::set(regs_->FMCFG, filter_bit);
  } else {
    emb::mmio::clear(regs_->FMCFG, filter_bit);
  }

  // set filter values
  regs_->sFilterRegister[filter_number].FBANK1 =
      (filter.filter_id_high << 16) | filter.filter_id_low;
  regs_->sFilterRegister[filter_number].FBANK2 =
      (filter.filter_mask_id_high << 16) | filter.filter_mask_id_low;

  // set FIFO assignment
  if (filter.fifo == rx_fifo::fifo1) {
    emb::mmio::set(regs_->FFASS, filter_bit);
  } else {
    emb::mmio::clear(regs_->FFASS, filter_bit);
  }

  // activate filter
  emb::mmio::set(regs_->FACT, filter_bit);

  // leave filter init mode
  emb::mmio::clear(regs_->FCTRL, CAN_FCTRL_FINITEN);

  return attr;
}

void peripheral::start() {
  emb::mmio::clear(regs_->MCTRL, CAN_MCTRL_INITREQ);
  emb::chrono::timeout start_t(std::chrono::milliseconds(2));
  while (emb::mmio::test_any(regs_->MSTS, CAN_MSTS_INITFLG)) {
    if (start_t.expired()) {
      while (true) {
        // TODO
      }
    }
  }
}

void peripheral::stop() {
  emb::mmio::set(regs_->MCTRL, CAN_MCTRL_INITREQ);
  emb::chrono::timeout stop_t(std::chrono::milliseconds(2));
  while (!emb::mmio::test_any(regs_->MSTS, CAN_MSTS_INITFLG)) {
    if (stop_t.expired()) {
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

  uint32_t mailboxid = emb::mmio::read(regs_->TXSTS, CAN_TXSTS_EMNUM);
  if (mailboxid > 2) {
    return exec_status::error;
  }

  // set up id
  if (frame.id <= 0x7FF) {
    regs_->sTxMailBox[mailboxid].TXMID = (frame.id << 21);
  } else if (frame.id <= 0x1FFFFFFF) {
    regs_->sTxMailBox[mailboxid].TXMID =
        (frame.id << 3) | CAN_TXMID0_IDTYPESEL;
  } else {
    return exec_status::invalid_argument;
  }

  // set up dlc
  emb::mmio::write(regs_->sTxMailBox[mailboxid].TXDLEN,
      CAN_TXDLEN0_DLCODE, frame.len & 0x0Fu);

  // set up data field
  regs_->sTxMailBox[mailboxid].TXMDL =
      (uint32_t(frame.payload[0]) << 0) | (uint32_t(frame.payload[1]) << 8) |
      (uint32_t(frame.payload[2]) << 16) | (uint32_t(frame.payload[3]) << 24);
  regs_->sTxMailBox[mailboxid].TXMDH =
      (uint32_t(frame.payload[4]) << 0) | (uint32_t(frame.payload[5]) << 8) |
      (uint32_t(frame.payload[6]) << 16) | (uint32_t(frame.payload[7]) << 24);

  // request transmission
  emb::mmio::set(regs_->sTxMailBox[mailboxid].TXMID, CAN_TXMID0_TXMREQ);

  return exec_status::ok;
}

std::optional<rxmessage_attr>
peripheral::get_frame(can_frame& frame, rx_fifo fifo) const {
  if (rxfifo_level(fifo) == 0) {
    return {};
  }

  auto fifo_idx = std::to_underlying(fifo);

  // get id
  if (!emb::mmio::test_any(regs_->sFIFOMailBox[fifo_idx].RXMID,
          CAN_RXMID0_IDTYPESEL)) {
    frame.id = regs_->sFIFOMailBox[fifo_idx].RXMID >> 21;
  } else {
    frame.id = regs_->sFIFOMailBox[fifo_idx].RXMID >> 3;
  }

  frame.len = uint8_t(emb::mmio::read(
      regs_->sFIFOMailBox[fifo_idx].RXDLEN, CAN_RXDLEN0_DLCODE));

  rxmessage_attr attr{};
  attr.filter_idx = emb::mmio::read(
      regs_->sFIFOMailBox[fifo_idx].RXDLEN, CAN_RXDLEN0_FMIDX);
  attr.fifo = fifo;

  // get data
  uint32_t const rdl = regs_->sFIFOMailBox[fifo_idx].RXMDL;
  uint32_t const rdh = regs_->sFIFOMailBox[fifo_idx].RXMDH;
  frame.payload[0] = uint8_t(rdl >> 0);
  frame.payload[1] = uint8_t(rdl >> 8);
  frame.payload[2] = uint8_t(rdl >> 16);
  frame.payload[3] = uint8_t(rdl >> 24);
  frame.payload[4] = uint8_t(rdh >> 0);
  frame.payload[5] = uint8_t(rdh >> 8);
  frame.payload[6] = uint8_t(rdh >> 16);
  frame.payload[7] = uint8_t(rdh >> 24);

  // release fifo
  switch (fifo) {
  case rx_fifo::fifo0:
    emb::mmio::set(regs_->RXF0, CAN_RXF0_RFOM0);
    break;
  case rx_fifo::fifo1:
    emb::mmio::set(regs_->RXF1, CAN_RXF1_RFOM1);
    break;
  }

  return {attr};
}

void peripheral::configure_interrupts(uint32_t interrupt_bitset) {
  emb::mmio::set(regs_->INTEN, interrupt_bitset);
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

  emb::mmio::set(RCM->APB1CLKEN, detail::clock_bits[can_idx]);
  is_clock_enabled_[can_idx] = true;
}

} // namespace can
} // namespace f4
} // namespace apm32
