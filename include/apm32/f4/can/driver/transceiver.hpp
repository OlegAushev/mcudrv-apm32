#pragma once

#include <apm32/f4/can/can_instances.hpp>
#include <apm32/f4/can/can_types.hpp>
#include <apm32/f4/can/can_utils.hpp>

#include <emb/can.hpp>
#include <emb/chrono.hpp>
#include <emb/concurrent.hpp>
#include <emb/delegate.hpp>
#include <emb/meta.hpp>

#include <expected>
#include <optional>

namespace apm32 {
namespace f4 {
namespace can {

inline constexpr size_t filter_count_total = 28;

struct transceiver_traits {
  size_t filter_count;
  size_t tx_queue_size;
};

struct transceiver_config {
  rx_pin_config rx_pin;
  tx_pin_config tx_pin;

  can::mode mode;

  uint16_t prescaler;      // 1..1024
  uint8_t sync_jump_width; // 1..4
  uint8_t time_segment1;   // 1..16
  uint8_t time_segment2;   // 1..8

  bool auto_bus_off_management;
  bool auto_wakeup;
  bool no_auto_retransmit;
  bool rx_fifo_locked;
  bool tx_fifo_priority;

  constexpr uint32_t bittim_reg() const {
    return ((prescaler - 1u) << CAN_BITTIM_BRPSC_Pos)
         | ((sync_jump_width - 1u) << CAN_BITTIM_RSYNJW_Pos)
         | ((time_segment1 - 1u) << CAN_BITTIM_TIMSEG1_Pos)
         | ((time_segment2 - 1u) << CAN_BITTIM_TIMSEG2_Pos)
         | ((std::to_underlying(mode) & 0b01u) << CAN_BITTIM_LBKMEN_Pos)
         | ((std::to_underlying(mode) >> 1) << CAN_BITTIM_SILMEN_Pos);
  }

  constexpr uint32_t mctrl_reg() const {
    return (tx_fifo_priority ? CAN_MCTRL_TXFPCFG : 0u)
         | (rx_fifo_locked ? CAN_MCTRL_RXFLOCK : 0u)
         | (no_auto_retransmit ? CAN_MCTRL_ARTXMD : 0u)
         | (auto_wakeup ? CAN_MCTRL_AWUPCFG : 0u)
         | (auto_bus_off_management ? CAN_MCTRL_ALBOFFM : 0u);
  }
};

template<some_can_instance Instance, transceiver_traits Traits>
class transceiver {
public:
  using can_instance = Instance;
  using token_type = uint32_t;
  using rx_delegate =
      emb::delegate<void(emb::canframe_t const&, token_type token)>;
private:
  static inline registers& REG = can_instance::REG;
  static constexpr nvic::irq_number rx0_irqn = can_instance::rx0_irqn;
  static constexpr nvic::irq_number rx1_irqn = can_instance::rx1_irqn;
  static constexpr nvic::irq_number tx_irqn = can_instance::tx_irqn;

  std::optional<gpio::alternate_pin> rx_pin_;
  std::optional<gpio::alternate_pin> tx_pin_;

  emb::isr_spsc_inplace_queue<emb::canframe_t, Traits.tx_queue_size> tx_queue_;
  rx_delegate on_rx_fifo0_;
  rx_delegate on_rx_fifo1_;
public:
  transceiver(transceiver_config const& config) {
    can_instance::enable_clock();

    // enter init mode
    emb::mmio::set(REG.MCTRL, CAN_MCTRL_INITREQ);
    emb::chrono::timeout init_timeout(std::chrono::milliseconds(2));
    while (!emb::mmio::test_any(REG.MSTS, CAN_MSTS_INITFLG)) {
      core::ensure(!init_timeout.expired());
    }

    // configure
    REG.MCTRL = (REG.MCTRL & CAN_MCTRL_INITREQ)
              | config.mctrl_reg(); // FIXME INITREQ?
    REG.BITTIM = config.bittim_reg();

    rx_pin_.emplace(
        gpio::alternate_pin_config{
            .port = config.rx_pin.port,
            .pin = config.rx_pin.pin,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = gpio::speed::medium,
            .altfunc = can_instance::gpio_altfunc
        }
    );

    tx_pin_.emplace(
        gpio::alternate_pin_config{
            .port = config.tx_pin.port,
            .pin = config.tx_pin.pin,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = gpio::speed::medium,
            .altfunc = can_instance::gpio_altfunc
        }
    );
  }

  void enable() {
    emb::mmio::clear(REG.MCTRL, CAN_MCTRL_INITREQ);
    emb::chrono::timeout start_timeout(std::chrono::milliseconds(2));
    while (emb::mmio::test_any(REG.MSTS, CAN_MSTS_INITFLG)) {
      core::ensure(!start_timeout.expired());
    }
  }

  std::optional<token_type> register_filter(
      emb::canid_t id,
      emb::canid_t mask,
      emb::canformat_t format,
      rx_fifo fifo
  ) {
    //
  }

  void on_rx_fifo0(rx_delegate sink) {
    on_rx_fifo0_ = sink;
  }

  void on_rx_fifo1(rx_delegate sink) {
    on_rx_fifo1_ = sink;
  }

  auto put_frame(emb::canframe_t const& frame) -> std::expected<void, error> {
    if (mailbox_full()) {
      return tx_queue_.try_push(frame) ? std::expected<void, error>{}
                                       : std::unexpected(error::overflow);
    }

    auto mailbox = emb::mmio::read(REG.TXSTS, CAN_TXSTS_EMNUM);
    if (mailbox > 2) {
      return std::unexpected(error::internal);
    }

    // ID
    uint32_t txmid;
    if (frame.format == emb::canformat_t::standard) {
      txmid = (frame.id << CAN_TXMID0_STDID_Pos);
    } else {
      txmid = (frame.id << CAN_TXMID0_EXTID_Pos) | CAN_TXMID0_IDTYPESEL;
    }
    REG.sTxMailBox[mailbox].TXMID = txmid;

    // DLC
    emb::mmio::write(
        REG.sTxMailBox[mailbox].TXDLEN,
        CAN_TXDLEN0_DLCODE,
        frame.len
    );

    // Data
    auto const words = std::bit_cast<std::array<uint32_t, 2>>(frame.payload);
    REG.sTxMailBox[mailbox].TXMDL = words[0];
    REG.sTxMailBox[mailbox].TXMDH = words[1];

    // Request transmission
    emb::mmio::set(REG.sTxMailBox[mailbox].TXMID, CAN_TXMID0_TXMREQ);

    return {};
  }

  template<rx_fifo RxFifo>
  auto get_frame() const
      -> std::optional<std::pair<emb::canframe_t, token_type>> {
    if (rx_messages_pending<RxFifo>() == 0) {
      return {};
    }

    constexpr auto fifo = std::to_underlying(RxFifo);

    token_type const token = emb::mmio::read(
        REG.sFIFOMailBox[fifo].RXDLEN,
        CAN_RXDLEN0_FMIDX
    );

    emb::canframe_t frame;

    if (!emb::mmio::test_any(
            REG.sFIFOMailBox[fifo].RXMID,
            CAN_RXMID0_IDTYPESEL
        )) {
      frame.format = emb::canformat_t::standard;
      frame.id = REG.sFIFOMailBox[fifo].RXMID >> 21;
    } else {
      frame.format = emb::canformat_t::extended;
      frame.id = REG.sFIFOMailBox[fifo].RXMID >> 3;
    }

    frame.len = uint8_t(
        emb::mmio::read(REG.sFIFOMailBox[fifo].RXDLEN, CAN_RXDLEN0_DLCODE)
    );

    frame.payload = std::bit_cast<canpayload_t>(
        std::array{REG.sFIFOMailBox[fifo].RXMDL, REG.sFIFOMailBox[fifo].RXMDH}
    );

    // Release FIFO
    if constexpr (RxFifo == rx_fifo::_0) {
      emb::mmio::set(REG.RXF0, CAN_RXF0_RFOM0);
    } else {
      emb::mmio::set(REG.RXF1, CAN_RXF1_RFOM1);
    }

    return std::make_pair(frame, token);
  }

private:
  bool mailbox_full() const {
    return !emb::mmio::test_any(REG.TXSTS, CAN_TXSTS_TXMEFLG);
  }

  template<rx_fifo RxFifo>
  size_t rx_messages_pending() const {
    if constexpr (RxFifo == rx_fifo::_0) {
      return emb::mmio::read(REG.RXF0, CAN_RXF0_FMNUM0);
    } else {
      return emb::mmio::read(REG.RXF1, CAN_RXF1_FMNUM1);
    }
  }
};

template<transceiver_traits Traits>
  requires(Traits.filter_count <= filter_count_total)
void init_filter_banks([[maybe_unused]] transceiver<can1, Traits>& can1_xcvr) {
  filter_init_session fg;
  emb::mmio::write(can1::REG.FCTRL, CAN_FCTRL_CAN2SB, Traits.filter_count);
}

template<transceiver_traits Traits>
  requires(Traits.filter_count <= filter_count_total)
void init_filter_banks([[maybe_unused]] transceiver<can2, Traits>& can2_xcvr) {
  can1::enable_clock();
  filter_init_session fg;
  emb::mmio::write(can1::REG.FCTRL, CAN_FCTRL_CAN2SB, 0u);
  emb::mmio::set(can1::REG.MCTRL, CAN_MCTRL_INITREQ);
}

template<transceiver_traits Traits1, transceiver_traits Traits2>
  requires(Traits1.filter_count + Traits2.filter_count <= filter_count_total)
void init_filter_banks(
    [[maybe_unused]] transceiver<can1, Traits1>& can1_xcvr,
    [[maybe_unused]] transceiver<can2, Traits2>& can2_xcvr
) {
  filter_init_session fg;
  emb::mmio::write(can1::REG.FCTRL, CAN_FCTRL_CAN2SB, Traits1.filter_count);
}

} // namespace can
} // namespace f4
} // namespace apm32
