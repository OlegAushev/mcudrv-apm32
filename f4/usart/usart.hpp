#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_usart.h>

#include <mcudrv-apm32/f4/apm32f4.hpp>
#include <mcudrv-apm32/f4/gpio/gpio.hpp>
#include <mcudrv-apm32/f4/system/system.hpp>

#include <emblib/mcudef.hpp>
#include <emblib/noncopyable.hpp>
#include <emblib/singleton.hpp>

namespace mcu {
namespace apm32 {
namespace usart {

using Regs = USART_T;

constexpr size_t periph_num{6};

enum class Peripheral : size_t {
  usart1,
  usart2,
  usart3,
  uart4,
  uart5,
  usart6,
};

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
  USART_Config_T hal_config;
};

namespace impl {

inline std::array<Regs*, periph_num> const regs = {
    USART1, USART2, USART3, UART4, UART5, USART6};

inline Peripheral get_peripheral(Regs const* reg) {
  return static_cast<Peripheral>(
      std::distance(regs.begin(), std::find(regs.begin(), regs.end(), reg)));
}

inline std::array<void (*)(void), periph_num> clk_enable_funcs = {
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_USART1); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USART2); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USART3); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_UART4); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_UART5); },
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_USART6); },
};

} // namespace impl

class Module : public mcu::uart::tty,
               public emb::singleton_array<Module, periph_num>,
               private emb::noncopyable {
private:
  Peripheral const peripheral_;
  USART_T* const regs_;
  gpio::AlternatePin rx_pin_;
  gpio::AlternatePin tx_pin_;

  static inline std::array<bool, periph_num> clk_enabled_{};
public:
  Module(Peripheral peripheral,
         RxPinConfig const& rx_pin_conf,
         TxPinConfig const& tx_pin_conf,
         Config const& conf);

  Peripheral peripheral() const { return peripheral_; }

  static Module* instance(Peripheral peripheral) {
    return emb::singleton_array<Module, periph_num>::instance(
        std::to_underlying(peripheral));
  }

  virtual int getchar() override {
    if (bit_is_clear<uint32_t>(regs_->STS, USART_FLAG_RXBNE)) {
      return EOF;
    }
    return regs_->DATA_B.DATA;
  }

  virtual int putchar(int ch) override {
    if (bit_is_clear<uint32_t>(regs_->STS, USART_FLAG_TXBE)) {
      return EOF;
    }
    regs_->DATA_B.DATA = static_cast<uint16_t>(ch) & 0x1FF;
    return ch;
  }

  // TODO
  // virtual int recv(char* buf, size_t len) override {
  //     int i = 0;
  //     char ch = 0;

  //     while ((i < len) && (getchar(ch) == 1)) {
  //         buf[i++] = ch;
  //     }
  //     return i;
  // }

  // TODO
  // virtual int send(const char* buf, size_t len) override {
  //     if (HAL_UART_Transmit(&_handle, reinterpret_cast<const uint8_t*>(buf), static_cast<uint16_t>(len), timeout_ms) != HAL_OK) {
  //         return 0;
  //     }
  //     return 1;
  // }
protected:
  static void enable_clk(Peripheral peripheral);
};

} // namespace usart
} // namespace apm32
} // namespace mcu

#endif
#endif
