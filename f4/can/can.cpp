#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv-apm32/f4/can/can.hpp>
#include <mcudrv-apm32/f4/chrono/chrono.hpp>

#include <emblib/chrono.hpp>

namespace mcu {
namespace apm32 {
namespace can {

Module::Module(Peripheral peripheral,
               const RxPinConfig& rx_pin_config,
               const TxPinConfig& tx_pin_config,
               Config config)
        : emb::singleton_array<Module, peripheral_count>(
                  this, std::to_underlying(peripheral)),
          _peripheral(peripheral) {
    _rx_pin.init({.port = rx_pin_config.port,
                  .pin = rx_pin_config.pin,
                  .config = {.pin{},
                             .mode = GPIO_MODE_AF,
                             .speed = GPIO_SPEED_50MHz,
                             .otype = GPIO_OTYPE_PP,
                             .pupd = GPIO_PUPD_NOPULL},
                  .altfunc = rx_pin_config.altfunc,
                  .active_state{}});

    _tx_pin.init({.port = tx_pin_config.port,
                  .pin = tx_pin_config.pin,
                  .config = {.pin{},
                             .mode = GPIO_MODE_AF,
                             .speed = GPIO_SPEED_50MHz,
                             .otype = GPIO_OTYPE_PP,
                             .pupd = GPIO_PUPD_NOPULL},
                  .altfunc = tx_pin_config.altfunc,
                  .active_state{}});

    _enable_clk(peripheral);
    _reg = impl::can_instances[static_cast<size_t>(_peripheral)];

    CAN_Config(_reg, &config.hal_config);
}

RxMessageAttribute Module::register_rxmessage(CAN_FilterConfig_T& filter) {
    RxMessageAttribute attr = {};

    if (_filter_count >= max_fitler_count) {
        fatal_error();
    }

    if (_peripheral == Peripheral::can1) {
        filter.filterNumber = _filter_count++;
    } else {
        filter.filterNumber = 14 + _filter_count++;
    }
    filter.filterActivation = ENABLE;
    //filter.SlaveStartFilterBank = 14;

    attr.filter_idx = filter.filterNumber;
    attr.fifo = RxFifo(filter.filterFIFO);

    CAN_ConfigFilter(&filter);

    return attr;
}

void Module::start() {
    _reg->MCTRL_B.INITREQ = 0;
    emb::chrono::watchdog start_wd(std::chrono::milliseconds(2));
    while (_reg->MSTS_B.INITFLG == 1) {
        if (!start_wd.good()) {
            fatal_error();
        }
    }
}

void Module::stop() {
    _reg->MCTRL_B.INITREQ = 1;
    emb::chrono::watchdog stop_wd(std::chrono::milliseconds(2));
    while (_reg->MSTS_B.INITFLG == 0) {
        if (!stop_wd.good()) {
            fatal_error();
        }
    }
}

exec_status Module::put_frame(const can_frame& frame) {
    if (mailbox_full()) {
        if (_txqueue.full()) {
            return exec_status::overflow;
        }
        _txqueue.push(frame);
        return exec_status::busy;
    }

    uint32_t mailboxid = _reg->TXSTS_B.EMNUM;
    if (mailboxid > 2) {
        return exec_status::error;
    }

    // set up id
    if (frame.id <= 0x7FF) {
        write_reg(_reg->sTxMailBox[mailboxid].TXMID, (frame.id << 21));
    } else if (frame.id <= 0x1FFFFFFF) {
        write_reg(_reg->sTxMailBox[mailboxid].TXMID, (frame.id << 3));
        _reg->sTxMailBox[mailboxid].TXMID_B.IDTYPESEL = 1;
    } else {
        return exec_status::invalid_argument;
    }

    // set up dlc
    _reg->sTxMailBox[mailboxid].TXDLEN_B.DLCODE = frame.len & 0x0F;

    // set up data field
    write_reg(_reg->sTxMailBox[mailboxid].TXMDL,
              (uint32_t(frame.payload[0]) << 0) |
                      (uint32_t(frame.payload[1]) << 8) |
                      (uint32_t(frame.payload[2]) << 16) |
                      (uint32_t(frame.payload[3]) << 24));
    write_reg(_reg->sTxMailBox[mailboxid].TXMDH,
              (uint32_t(frame.payload[4]) << 0) |
                      (uint32_t(frame.payload[5]) << 8) |
                      (uint32_t(frame.payload[6]) << 16) |
                      (uint32_t(frame.payload[7]) << 24));

    // request transmission
    _reg->sTxMailBox[mailboxid].TXMID_B.TXMREQ = 1;

    return exec_status::ok;
}

std::optional<RxMessageAttribute> Module::get_frame(can_frame& frame,
                                                    RxFifo fifo) const {
    if (rxfifo_level(fifo) == 0) {
        return {};
    }

    auto fifo_idx = std::to_underlying(fifo);

    // get id, len, filter
    if (_reg->sRxMailBox[fifo_idx].RXMID_B.IDTYPESEL == 0) {
        frame.id = _reg->sRxMailBox[fifo_idx].RXMID >> 21;
    } else {
        frame.id = _reg->sRxMailBox[fifo_idx].RXMID >> 3;
    }

    frame.len = uint8_t(_reg->sRxMailBox[fifo_idx].RXDLEN_B.DLCODE);

    RxMessageAttribute attr{};
    attr.filter_idx = _reg->sRxMailBox[fifo_idx].RXDLEN_B.FMIDX;
    attr.fifo = fifo;

    // get data
    frame.payload[0] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE0);
    frame.payload[1] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE1);
    frame.payload[2] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE2);
    frame.payload[3] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDL_B.DATABYTE3);
    frame.payload[4] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE4);
    frame.payload[5] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE5);
    frame.payload[6] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE6);
    frame.payload[7] = uint8_t(_reg->sRxMailBox[fifo_idx].RXMDH_B.DATABYTE7);

    // release fifo
    switch (fifo) {
    case RxFifo::fifo0:
        _reg->RXF0_B.RFOM0 = 1;
        break;
    case RxFifo::fifo1:
        _reg->RXF1_B.RFOM1 = 1;
        break;
    }

    return {attr};
}

void Module::init_interrupts(uint32_t interrupt_bitset) {
    set_bit(_reg->INTEN, interrupt_bitset);
}

void Module::set_interrupt_priority(IrqPriority rx0_priority,
                                    IrqPriority rx1_priority,
                                    IrqPriority tx_priority) {
    set_irq_priority(impl::can_rx0_irqn[std::to_underlying(_peripheral)],
                     rx0_priority);
    set_irq_priority(impl::can_rx1_irqn[std::to_underlying(_peripheral)],
                     rx1_priority);
    set_irq_priority(impl::can_tx_irqn[std::to_underlying(_peripheral)],
                     tx_priority);
}

void Module::enable_interrupts() {
    enable_irq(impl::can_rx0_irqn[std::to_underlying(_peripheral)]);
    enable_irq(impl::can_rx1_irqn[std::to_underlying(_peripheral)]);
    enable_irq(impl::can_tx_irqn[std::to_underlying(_peripheral)]);
}

void Module::disable_interrupts() {
    disable_irq(impl::can_rx0_irqn[std::to_underlying(_peripheral)]);
    disable_irq(impl::can_rx1_irqn[std::to_underlying(_peripheral)]);
    disable_irq(impl::can_tx_irqn[std::to_underlying(_peripheral)]);
}

void Module::_enable_clk(Peripheral peripheral) {
    auto can_idx = std::to_underlying(peripheral);
    if (_clk_enabled[can_idx]) {
        return;
    }

    impl::can_clk_enable_funcs[can_idx]();
    _clk_enabled[can_idx] = true;
}

} // namespace can
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
#endif
