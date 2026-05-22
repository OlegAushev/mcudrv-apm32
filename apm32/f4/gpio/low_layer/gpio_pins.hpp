#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/gpio/low_layer/gpio_ports.hpp>

#include <emb/gpio.hpp>

#include <cstdint>

namespace apm32::f4::gpio {

enum class pin : std::uint16_t {
  pin0 = (1u << 0),
  pin1 = (1u << 1),
  pin2 = (1u << 2),
  pin3 = (1u << 3),
  pin4 = (1u << 4),
  pin5 = (1u << 5),
  pin6 = (1u << 6),
  pin7 = (1u << 7),
  pin8 = (1u << 8),
  pin9 = (1u << 9),
  pin10 = (1u << 10),
  pin11 = (1u << 11),
  pin12 = (1u << 12),
  pin13 = (1u << 13),
  pin14 = (1u << 14),
  pin15 = (1u << 15),
};

enum class output_type : std::uint32_t {
  pushpull = 0b0,
  opendrain = 0b1
};

enum class speed : std::uint32_t {
  low = 0b00,
  medium = 0b01,
  // high = 0b10,
  very_high = 0b11
};

enum class pull : std::uint32_t {
  none = 0b00,
  up = 0b01,
  down = 0b10
};

namespace mode {
inline constexpr std::uint32_t input = 0b00;
inline constexpr std::uint32_t output = 0b01;
inline constexpr std::uint32_t alternate = 0b10;
inline constexpr std::uint32_t analog = 0b11;
} // namespace mode

namespace altfunc {
inline constexpr std::uint32_t tmr1 = 1;
inline constexpr std::uint32_t tmr2 = 1;
inline constexpr std::uint32_t tmr3 = 2;
inline constexpr std::uint32_t tmr4 = 2;
inline constexpr std::uint32_t tmr5 = 2;
inline constexpr std::uint32_t tmr8 = 3;
inline constexpr std::uint32_t tmr9 = 3;
inline constexpr std::uint32_t tmr10 = 3;
inline constexpr std::uint32_t tmr11 = 3;
inline constexpr std::uint32_t i2c1 = 4;
inline constexpr std::uint32_t i2c2 = 4;
inline constexpr std::uint32_t i2c3 = 4;
inline constexpr std::uint32_t spi1 = 5;
inline constexpr std::uint32_t spi2 = 5;
inline constexpr std::uint32_t spi3 = 6;
inline constexpr std::uint32_t usart1 = 7;
inline constexpr std::uint32_t usart2 = 7;
inline constexpr std::uint32_t usart3 = 7;
inline constexpr std::uint32_t uart4 = 8;
inline constexpr std::uint32_t uart5 = 8;
inline constexpr std::uint32_t usart6 = 8;
inline constexpr std::uint32_t can1 = 9;
inline constexpr std::uint32_t can2 = 9;
inline constexpr std::uint32_t tmr12 = 9;
inline constexpr std::uint32_t tmr13 = 9;
inline constexpr std::uint32_t tmr14 = 9;
} // namespace altfunc

struct input_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
  apm32::f4::gpio::pull pull;
  emb::gpio::polarity polarity;
};

struct output_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
  apm32::f4::gpio::pull pull;
  apm32::f4::gpio::output_type output_type;
  apm32::f4::gpio::speed speed;
  emb::gpio::polarity polarity;
};

struct alternate_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
  apm32::f4::gpio::pull pull;
  apm32::f4::gpio::output_type output_type;
  apm32::f4::gpio::speed speed;
  std::uint32_t altfunc;
};

struct analog_pin_config {
  apm32::f4::gpio::port port;
  apm32::f4::gpio::pin pin;
};

} // namespace apm32::f4::gpio
