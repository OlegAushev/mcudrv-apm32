#pragma once

#include <apm32/f4/spi/low_layer/spi_instances.hpp>

#include <emb/mmio.hpp>
#include <emb/units.hpp>

#include <cstdint>

namespace apm32::f4::spi {

enum class error {
  timeout,
  overrun,
  underrun,
  mode_fault,
  crc_error,
  frame_error
};

struct mosi_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct miso_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct clk_pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct ss_pin_config {
  gpio::port port;
  gpio::pin pin;
};

enum class clock_polarity : std::uint32_t { low = 0, high = 1 };

enum class clock_phase : std::uint32_t { first_edge = 0, second_edge = 1 };

enum class data_length : std::uint32_t { bits_8 = 0, bits_16 = 1 };

enum class bit_order : std::uint32_t { msb_first = 0, lsb_first = 1 };

enum class baudrate_prescaler : std::uint32_t {
  div2 = 0b000,
  div4 = 0b001,
  div8 = 0b010,
  div16 = 0b011,
  div32 = 0b100,
  div64 = 0b101,
  div128 = 0b110,
  div256 = 0b111,
};

template<typename T>
concept frame_format = emb::same_as_any<T, std::uint8_t, std::uint16_t>;

} // namespace apm32::f4::spi
