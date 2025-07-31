#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_dac.h>

#include <mcu/apm32/f4/gpio.hpp>
#include <mcu/apm32/f4/system.hpp>

#include <emb/noncopyable.hpp>
#include <emb/singleton.hpp>

#include <utility>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace dac {

using Regs = DAC_T;

constexpr size_t periph_num = 1;

enum class Peripheral : size_t {
  dac1
};

enum class Channel : uint32_t {
  channel1 = DAC_CHANNEL_1,
  channel2 = DAC_CHANNEL_2
};

enum class DataAlignment : uint32_t {
  right_12bit = DAC_ALIGN_12BIT_R,
  left_12bit = DAC_ALIGN_12BIT_L,
  right_8bit = DAC_ALIGN_8BIT_R
};

struct PinConfig {
  gpio::Port port;
  gpio::Pin pin;
};

struct ChannelConfig {
  DAC_Config_T hal_config;
};

namespace detail {

inline std::array<Regs*, periph_num> const regs{DAC};

inline Peripheral get_peripheral(const DAC_T* reg) {
  return static_cast<Peripheral>(
      std::distance(regs.begin(), std::find(regs.begin(), regs.end(), reg)));
}

inline std::array<void (*)(void), periph_num> clk_enable_funcs = {
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_DAC); },
};

} // namespace detail

class Module : public emb::singleton_array<Module, periph_num>,
               private emb::noncopyable {
private:
  Peripheral const peripheral_;
  Regs* const regs_;
  static inline std::array<bool, periph_num> clk_enabled_{};
public:
  Module(Peripheral peripheral);

  [[nodiscard]] std::unique_ptr<gpio::AnalogPin> initialize_channel(
      Channel channel, PinConfig const& pinconf, ChannelConfig const& conf);

  Peripheral peripheral() const { return peripheral_; }

  Regs* reg() { return regs_; }

  static Module* instance(Peripheral peripheral) {
    return emb::singleton_array<Module, periph_num>::instance(
        std::to_underlying(peripheral));
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

protected:
  static void enable_clk(Peripheral peripheral);
};

} // namespace dac
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
#endif
