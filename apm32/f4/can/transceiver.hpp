#pragma once

#include <apm32/f4/can/can.hpp>

#include <apm32/f4/chrono/chrono.hpp>
#include <apm32/f4/gpio/alternate_pin.hpp>

#include <emb/assert.hpp>
#include <emb/can.hpp>
#include <emb/chrono.hpp>
#include <emb/concurrent/isr_spsc_inplace_queue.hpp>
#include <emb/delegate.hpp>
#include <emb/meta.hpp>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <utility>

namespace apm32::f4::can {

inline constexpr std::size_t filter_count_total = 28;

struct transceiver_traits {
  std::size_t filter_count;
  std::size_t tx_queue_size;
};

struct transceiver_config {
  rx_pin_config rx_pin;
  tx_pin_config tx_pin;

  can::mode mode;

  std::uint16_t prescaler;      // 1..1024
  std::uint8_t sync_jump_width; // 1..4
  std::uint8_t time_segment1;   // 1..16
  std::uint8_t time_segment2;   // 1..8

  bool auto_bus_off_management;
  bool auto_wakeup;
  bool no_auto_retransmit;
  bool rx_fifo_locked;
  bool tx_fifo_priority;

  nvic::irq_priority rx_fifo0_irq_priority;
  nvic::irq_priority rx_fifo1_irq_priority;
  nvic::irq_priority tx_irq_priority;
  nvic::irq_priority sce_irq_priority;

  constexpr std::uint32_t bittim_reg() const {
    return ((prescaler - 1u) << CAN_BITTIM_BRPSC_Pos)
         | ((sync_jump_width - 1u) << CAN_BITTIM_RSYNJW_Pos)
         | ((time_segment1 - 1u) << CAN_BITTIM_TIMSEG1_Pos)
         | ((time_segment2 - 1u) << CAN_BITTIM_TIMSEG2_Pos)
         | ((std::to_underlying(mode) & 0b01u) << CAN_BITTIM_LBKMEN_Pos)
         | ((std::to_underlying(mode) >> 1) << CAN_BITTIM_SILMEN_Pos);
  }

  constexpr std::uint32_t mctrl_reg() const {
    return (tx_fifo_priority ? CAN_MCTRL_TXFPCFG : 0u)
         | (rx_fifo_locked ? CAN_MCTRL_RXFLOCK : 0u)
         | (no_auto_retransmit ? CAN_MCTRL_ARTXMD : 0u)
         | (auto_wakeup ? CAN_MCTRL_AWUPCFG : 0u)
         | (auto_bus_off_management ? CAN_MCTRL_ALBOFFM : 0u);
  }
};

template<some_can_instance Instance, transceiver_traits Traits>
class transceiver;

template<some_can_instance Instance, transceiver_traits Traits>
class filter_setup;

template<transceiver_traits Traits>
  requires(Traits.filter_count <= filter_count_total)
[[nodiscard]] auto init_filter_banks(transceiver<can1, Traits>& can1_xcvr)
    -> filter_setup<can1, Traits>;

template<transceiver_traits Traits>
  requires(Traits.filter_count <= filter_count_total)
[[nodiscard]] auto init_filter_banks(transceiver<can2, Traits>& can2_xcvr)
    -> filter_setup<can2, Traits>;

template<transceiver_traits Traits1, transceiver_traits Traits2>
  requires(Traits1.filter_count + Traits2.filter_count <= filter_count_total)
[[nodiscard]] auto init_filter_banks(
    transceiver<can1, Traits1>& can1_xcvr,
    transceiver<can2, Traits2>& can2_xcvr
) -> std::pair<filter_setup<can1, Traits1>, filter_setup<can2, Traits2>>;

template<some_can_instance Instance, transceiver_traits Traits>
class transceiver {
  template<some_can_instance, transceiver_traits>
  friend class filter_setup;
public:
  using can_instance = Instance;
  using rx_delegate = emb::delegate<void(emb::can::frame_t const&)>;
private:
  using timeout_t = emb::chrono::timeout<chrono::steady_clock>;

  static inline registers& reg = Instance::reg;
  static constexpr nvic::irq_number rx0_irqn = Instance::rx0_irqn;
  static constexpr nvic::irq_number rx1_irqn = Instance::rx1_irqn;
  static constexpr nvic::irq_number tx_irqn = Instance::tx_irqn;
  static constexpr nvic::irq_number sce_irqn = Instance::sce_irqn;

  std::optional<gpio::alternate_pin> rx_pin_;
  std::optional<gpio::alternate_pin> tx_pin_;

  std::uint32_t filters_used_ = 0;

  emb::isr_spsc_inplace_queue<emb::can::frame_t, Traits.tx_queue_size>
      tx_queue_;
  rx_delegate on_rx_fifo0_;
  rx_delegate on_rx_fifo1_;
public:
  transceiver(transceiver const&) = delete;
  transceiver& operator=(transceiver const&) = delete;
  transceiver(transceiver&&) = delete;
  transceiver& operator=(transceiver&&) = delete;

  transceiver(transceiver_config const& config) {
    Instance::enable_clock();

    // enter init mode
    emb::mmio::set(reg.MCTRL, CAN_MCTRL_INITREQ);
    timeout_t init_timeout(std::chrono::milliseconds(2));
    while (!emb::mmio::test_any(reg.MSTS, CAN_MSTS_INITFLG)) {
      emb::ensure(!init_timeout.expired());
    }

    // exit sleep mode
    emb::mmio::clear(reg.MCTRL, CAN_MCTRL_SLEEPREQ);
    timeout_t sleep_timeout(std::chrono::milliseconds(2));
    while (emb::mmio::test_any(reg.MSTS, CAN_MSTS_SLEEPFLG)) {
      emb::ensure(!sleep_timeout.expired());
    }

    // configure
    reg.MCTRL = reg.MCTRL | config.mctrl_reg();
    reg.BITTIM = config.bittim_reg();

    rx_pin_.emplace(
        gpio::alternate_pin_config{
            .port = config.rx_pin.port,
            .pin = config.rx_pin.pin,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = gpio::speed::medium,
            .altfunc = Instance::gpio_altfunc
        }
    );

    tx_pin_.emplace(
        gpio::alternate_pin_config{
            .port = config.tx_pin.port,
            .pin = config.tx_pin.pin,
            .pull = gpio::pull::none,
            .output_type = gpio::output_type::pushpull,
            .speed = gpio::speed::medium,
            .altfunc = Instance::gpio_altfunc
        }
    );

    Instance::on_irq_rx0 = emb::make_delegate<&transceiver::on_irq_rx0>(this);
    Instance::on_irq_rx1 = emb::make_delegate<&transceiver::on_irq_rx1>(this);
    Instance::on_irq_tx = emb::make_delegate<&transceiver::on_irq_tx>(this);
    Instance::on_irq_sce = emb::make_delegate<&transceiver::on_irq_sce>(this);

    // interrupt configuration
    emb::mmio::set(
        reg.INTEN,
        CAN_INTEN_FMIEN0 | CAN_INTEN_FMIEN1 | CAN_INTEN_TXMEIEN
    );
    nvic::set_irq_priority(rx0_irqn, config.rx_fifo0_irq_priority);
    nvic::set_irq_priority(rx1_irqn, config.rx_fifo1_irq_priority);
    nvic::set_irq_priority(tx_irqn, config.tx_irq_priority);
    nvic::set_irq_priority(sce_irqn, config.sce_irq_priority);
  }

  void enable() {
    nvic::clear_pending_irq(rx0_irqn);
    nvic::enable_irq(rx0_irqn);
    nvic::clear_pending_irq(rx1_irqn);
    nvic::enable_irq(rx1_irqn);
    nvic::clear_pending_irq(tx_irqn);
    nvic::enable_irq(tx_irqn);
    nvic::clear_pending_irq(sce_irqn);
    // TODO nvic::enable_irq(sce_irqn);

    emb::mmio::clear(reg.MCTRL, CAN_MCTRL_INITREQ);
    timeout_t start_timeout(std::chrono::milliseconds(2));
    while (emb::mmio::test_any(reg.MSTS, CAN_MSTS_INITFLG)) {
      emb::ensure(!start_timeout.expired());
    }
  }

  void on_rx_fifo0(rx_delegate sink) {
    on_rx_fifo0_ = sink;
  }

  void on_rx_fifo1(rx_delegate sink) {
    on_rx_fifo1_ = sink;
  }

  auto put(emb::can::frame_t const& frame) -> std::expected<void, error> {
    if (!tx_queue_.try_push(frame)) return std::unexpected(error::overflow);
    if (!all_mailboxes_busy()) nvic::set_pending_irq(tx_irqn);
    return {};
  }

  template<rx_fifo RxFifo>
  auto get() -> std::optional<emb::can::frame_t> {
    if (rx_messages_pending<RxFifo>() == 0) {
      return {};
    }

    constexpr auto fifo = std::to_underlying(RxFifo);
    emb::can::frame_t frame;

    std::uint32_t const rxmid = reg.sFIFOMailBox[fifo].RXMID;
    if (!emb::mmio::test_any(rxmid, CAN_RXMID0_IDTYPESEL)) {
      frame.format = emb::can::format_t::standard;
      frame.id = rxmid >> 21;
    } else {
      frame.format = emb::can::format_t::extended;
      frame.id = rxmid >> 3;
    }

    frame.len = std::uint8_t(
        emb::mmio::read<CAN_RXDLEN0_DLCODE>(reg.sFIFOMailBox[fifo].RXDLEN)
    );

    frame.payload = std::bit_cast<emb::can::payload_t>(
        std::array{reg.sFIFOMailBox[fifo].RXMDL, reg.sFIFOMailBox[fifo].RXMDH}
    );

    // release FIFO
    if constexpr (RxFifo == rx_fifo::_0) {
      emb::mmio::set(reg.RXF0, CAN_RXF0_RFOM0);
    } else {
      emb::mmio::set(reg.RXF1, CAN_RXF1_RFOM1);
    }

    return frame;
  }

private:
  std::uint32_t get_next_filter_idx() {
    std::uint32_t bank_offset = 0;
    if constexpr (std::same_as<Instance, can2>) {
      bank_offset = emb::mmio::read<CAN_FCTRL_CAN2SB>(can1::reg.FCTRL);
    }
    return bank_offset + filters_used_;
  }

  void add_filter(filter_32_mask const& filter, rx_fifo fifo) {
    emb::ensure(filters_used_ < Traits.filter_count);

    setup_filter_bank(
        filter_scale::_32bit,
        filter_mode::mask,
        fifo,
        get_next_filter_idx(),
        detail::encode_32bit_id(filter.format, filter.id),
        detail::encode_32bit_mask(filter.format, filter.mask)
    );

    ++filters_used_;
  }

  void add_filter(filter_32_list const& filter, rx_fifo fifo) {
    emb::ensure(filters_used_ < Traits.filter_count);

    setup_filter_bank(
        filter_scale::_32bit,
        filter_mode::list,
        fifo,
        get_next_filter_idx(),
        detail::encode_32bit_id(filter.format, filter.id1),
        detail::encode_32bit_id(filter.format, filter.id2)
    );

    ++filters_used_;
  }

  void add_filter(filter_16_mask const& filter, rx_fifo fifo) {
    emb::ensure(filters_used_ < Traits.filter_count);

    std::uint32_t const bank1 = detail::encode_16bit_mask(filter.mask1) << 16
                         | detail::encode_16bit_id(filter.id1);
    std::uint32_t const bank2 = detail::encode_16bit_mask(filter.mask2) << 16
                         | detail::encode_16bit_id(filter.id2);

    setup_filter_bank(
        filter_scale::_16bit,
        filter_mode::mask,
        fifo,
        get_next_filter_idx(),
        bank1,
        bank2
    );

    ++filters_used_;
  }

  void add_filter(filter_16_list const& filter, rx_fifo fifo) {
    emb::ensure(filters_used_ < Traits.filter_count);

    std::uint32_t const bank1 = detail::encode_16bit_id(filter.id2) << 16
                         | detail::encode_16bit_id(filter.id1);
    std::uint32_t const bank2 = detail::encode_16bit_id(filter.id4) << 16
                         | detail::encode_16bit_id(filter.id3);

    setup_filter_bank(
        filter_scale::_16bit,
        filter_mode::list,
        fifo,
        get_next_filter_idx(),
        bank1,
        bank2
    );

    ++filters_used_;
  }

  bool all_mailboxes_busy() const {
    return !emb::mmio::test_any(reg.TXSTS, CAN_TXSTS_TXMEFLG);
  }

  template<rx_fifo RxFifo>
  std::size_t rx_messages_pending() const {
    if constexpr (RxFifo == rx_fifo::_0) {
      return emb::mmio::read<CAN_RXF0_FMNUM0>(reg.RXF0);
    } else {
      return emb::mmio::read<CAN_RXF1_FMNUM1>(reg.RXF1);
    }
  }

  void populate_mailboxes() {
    while (!all_mailboxes_busy()) {
      auto frame = tx_queue_.try_pop();
      if (!frame) return;
      put_into_mailbox(*frame);
    }
  }

  void put_into_mailbox(emb::can::frame_t const& frame) {
    auto mailbox = emb::mmio::read<CAN_TXSTS_EMNUM>(reg.TXSTS);
    if (mailbox > 2) return;

    // ID
    std::uint32_t txmid;
    if (frame.format == emb::can::format_t::standard) {
      txmid = (frame.id << CAN_TXMID0_STDID_Pos);
    } else {
      txmid = (frame.id << CAN_TXMID0_EXTID_Pos) | CAN_TXMID0_IDTYPESEL;
    }
    reg.sTxMailBox[mailbox].TXMID = txmid;

    // DLC
    emb::mmio::write<CAN_TXDLEN0_DLCODE>(
        reg.sTxMailBox[mailbox].TXDLEN,
        frame.len
    );

    // data
    auto const words = std::bit_cast<std::array<std::uint32_t, 2>>(frame.payload);
    reg.sTxMailBox[mailbox].TXMDL = words[0];
    reg.sTxMailBox[mailbox].TXMDH = words[1];

    // request transmission
    emb::mmio::set(reg.sTxMailBox[mailbox].TXMID, CAN_TXMID0_TXMREQ);
  }

private:
  void on_irq_rx0() {
    while (auto frame = get<rx_fifo::_0>()) {
      if (on_rx_fifo0_) on_rx_fifo0_(*frame);
    }
  }

  void on_irq_rx1() {
    while (auto frame = get<rx_fifo::_1>()) {
      if (on_rx_fifo1_) on_rx_fifo1_(*frame);
    }
  }

  void on_irq_tx() {
    emb::mmio::clear_w1(
        reg.TXSTS,
        CAN_TXSTS_REQCFLG0 | CAN_TXSTS_REQCFLG1 | CAN_TXSTS_REQCFLG2
    );

    populate_mailboxes();
  }

  void on_irq_sce() {
    // TODO
  }
};

// Phase guard for filter configuration: only `init_filter_banks` can construct
// a `filter_setup`, so `add_filter` is unreachable until CAN2SB has been
// programmed.
template<some_can_instance Instance, transceiver_traits Traits>
class filter_setup {
  transceiver<Instance, Traits>& xcvr_;

  explicit filter_setup(transceiver<Instance, Traits>& xcvr) : xcvr_(xcvr) {}

  template<transceiver_traits T>
    requires(T.filter_count <= filter_count_total)
  friend auto init_filter_banks(transceiver<can1, T>& can1_xcvr)
      -> filter_setup<can1, T>;

  template<transceiver_traits T>
    requires(T.filter_count <= filter_count_total)
  friend auto init_filter_banks(transceiver<can2, T>& can2_xcvr)
      -> filter_setup<can2, T>;

  template<transceiver_traits T1, transceiver_traits T2>
    requires(T1.filter_count + T2.filter_count <= filter_count_total)
  friend auto init_filter_banks(
      transceiver<can1, T1>& can1_xcvr,
      transceiver<can2, T2>& can2_xcvr
  ) -> std::pair<filter_setup<can1, T1>, filter_setup<can2, T2>>;

public:
  void add(filter_32_mask const& filter, rx_fifo fifo) {
    xcvr_.add_filter(filter, fifo);
  }

  void add(filter_32_list const& filter, rx_fifo fifo) {
    xcvr_.add_filter(filter, fifo);
  }

  void add(filter_16_mask const& filter, rx_fifo fifo) {
    xcvr_.add_filter(filter, fifo);
  }

  void add(filter_16_list const& filter, rx_fifo fifo) {
    xcvr_.add_filter(filter, fifo);
  }
};

template<transceiver_traits Traits>
  requires(Traits.filter_count <= filter_count_total)
[[nodiscard]] auto init_filter_banks(transceiver<can1, Traits>& can1_xcvr)
    -> filter_setup<can1, Traits> {
  filter_init_session fg;
  emb::mmio::write<CAN_FCTRL_CAN2SB>(can1::reg.FCTRL, Traits.filter_count);
  return filter_setup<can1, Traits>{can1_xcvr};
}

template<transceiver_traits Traits>
  requires(Traits.filter_count <= filter_count_total)
[[nodiscard]] auto init_filter_banks(transceiver<can2, Traits>& can2_xcvr)
    -> filter_setup<can2, Traits> {
  can1::enable_clock();
  filter_init_session fg;
  emb::mmio::write<CAN_FCTRL_CAN2SB>(can1::reg.FCTRL, 0u);
  // leave CAN1 in init mode
  emb::mmio::set(can1::reg.MCTRL, CAN_MCTRL_INITREQ);
  return filter_setup<can2, Traits>{can2_xcvr};
}

template<transceiver_traits Traits1, transceiver_traits Traits2>
  requires(Traits1.filter_count + Traits2.filter_count <= filter_count_total)
[[nodiscard]] auto init_filter_banks(
    transceiver<can1, Traits1>& can1_xcvr,
    transceiver<can2, Traits2>& can2_xcvr
) -> std::pair<filter_setup<can1, Traits1>, filter_setup<can2, Traits2>> {
  filter_init_session fg;
  emb::mmio::write<CAN_FCTRL_CAN2SB>(can1::reg.FCTRL, Traits1.filter_count);
  return {
      filter_setup<can1, Traits1>{can1_xcvr},
      filter_setup<can2, Traits2>{can2_xcvr}
  };
}

} // namespace apm32::f4::can
