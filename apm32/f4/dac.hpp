#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32/f4/core.hpp>
#include <apm32/f4/gpio.hpp>

#include <emb/mmio.hpp>

#include <algorithm>
#include <utility>

namespace apm32 {
namespace f4 {
namespace dac {

using peripheral_registers = DAC_TypeDef;

inline constexpr size_t peripheral_count = 1;

inline std::array<peripheral_registers*, peripheral_count> const peripherals = {
    DAC
};

enum class peripheral_id : uint32_t { dac1 };

// channel offset in CTRL register: ch1 starts at bit 0, ch2 at bit 16
enum class channel : uint32_t { ch1 = 0, ch2 = 16 };

struct pin_config {
  gpio::port port;
  gpio::pin pin;
};

struct channel_config {
  bool output_buffer_disable;
  uint32_t trigger_selection;
  bool trigger_enable;
};

namespace detail {

inline constexpr std::array<uint32_t, peripheral_count> clock_bits = {
    RCM_APB1CLKEN_DACEN,
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

private:
  static void enable_clock(peripheral_id id);
};

} // namespace dac
} // namespace f4
} // namespace apm32
