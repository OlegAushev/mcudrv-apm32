#pragma once

#ifdef APM32F4XX

#include <apm32f4xx_usart.h>

#include <mcu/apm32/f4/gpio.hpp>
#include <mcu/apm32/f4/system.hpp>

#include <emb/noncopyable.hpp>
#include <emb/singleton.hpp>
#include <emb/uart.hpp>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
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

inline std::array<Regs*, periph_num> const regs = {
    USART1, USART2, USART3, UART4, UART5, USART6};

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
  USART_Config_T hal_config;
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

} // namespace internal

class Module : public emb::uart::tty,
               public emb::singleton_array<Module, periph_num>,
               private emb::noncopyable {
private:
  Peripheral const peripheral_;
  USART_T* const regs_;
  internal::RxPin rx_pin_;
  internal::TxPin tx_pin_;
public:
  Module(Peripheral peripheral,
         RxPinConfig const& rx_pinconf,
         TxPinConfig const& tx_pinconf,
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
  static inline std::array<bool, periph_num> clk_enabled_{};
  static void enable_clk(Peripheral peripheral);
  static inline std::array<void (*)(void), periph_num> enable_clk_ = {
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_USART1); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USART2); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USART3); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_UART4); },
      []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_UART5); },
      []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_USART6); }};
};

} // namespace usart
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
