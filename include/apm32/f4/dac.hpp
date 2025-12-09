#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>

#include <apm32f4xx_dac.h>

#include <algorithm>
#include <utility>

namespace apm32 {
namespace f4 {
namespace dac {

using peripheral_registers = DAC_T;

inline constexpr size_t peripheral_count = 1;

inline std::array<peripheral_registers*, peripheral_count> const peripherals = {
    DAC
};

enum class peripheral_id : uint32_t { dac1 };

enum class channel : uint32_t { ch1 = DAC_CHANNEL_1, ch2 = DAC_CHANNEL_2 };

enum class data_alignment : uint32_t {
  right_12bit = DAC_ALIGN_12BIT_R,
  left_12bit = DAC_ALIGN_12BIT_L,
  right_8bit = DAC_ALIGN_8BIT_R
};

struct pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct channel_config {
  DAC_Config_T hal_config;
};

namespace detail {

inline std::array<void (*)(void), peripheral_count> const enable_clock = {
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_DAC); },
};

} // namespace detail

class peripheral
    : public core::peripheral<peripheral, peripheral_id, peripheral_count> {
public:
  using peripheral_type =
      core::peripheral<peripheral, peripheral_id, peripheral_count>;
private:
  peripheral_id const id_;
  peripheral_registers* const regs_;
  static inline std::array<bool, peripheral_count> is_clock_enabled_{};
public:
  peripheral(peripheral_id id);

  [[nodiscard]] std::unique_ptr<gpio::analog_pin> configure_channel(
      channel ch,
      pin_config const& pinconf,
      channel_config chconf
  );

  peripheral_id id() const {
    return id_;
  }

  peripheral_registers* regs() {
    return regs_;
  }

  // void convert(Channel channel, DataAlignment alignment, uint32_t value) {
  //     uint32_t tmp = reinterpret_cast<uint32_t>(_handle.Instance);
  //     if (channel == Channel::channel1) {
  //         tmp += DAC_DHR12R1_ALIGNMENT(std::to_underlying(alignment));
  //     } else {
  //         tmp += DAC_DHR12R2_ALIGNMENT(std::to_underlying(alignment));
  //     }

  //     *(reinterpret_cast<volatile uint32_t *>(tmp)) = value;
  // }

private:
  static void enable_clock(peripheral_id id);
};

} // namespace dac
} // namespace f4
} // namespace apm32
