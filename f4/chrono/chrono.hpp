#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_misc.h>

#include <mcudrv-apm32/f4/apm32f4.hpp>
#include <mcudrv-apm32/f4/system/system.hpp>

#include <array>
#include <chrono>
#include <emblib/static_vector.hpp>

extern "C" void SysTick_Handler();

namespace mcu {
inline namespace apm32 {
namespace chrono {

class steady_clock {
  friend void ::SysTick_Handler();
public:
  using duration = std::chrono::milliseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<steady_clock, duration>;
  static constexpr bool is_steady = true;
private:
  static inline bool initialized_{false};
  static inline int64_t volatile time_{0};
  static constexpr std::chrono::milliseconds time_step{1};
public:
  steady_clock() = delete;
  static void init();

  static bool initialized() { return initialized_; }

  static std::chrono::time_point<steady_clock> now() {
    return time_point{std::chrono::milliseconds{time_}};
  }

  static std::chrono::milliseconds step() { return time_step; }

  static void delay(std::chrono::milliseconds delay) {
    auto const start{now()};
    while ((now() - start) <= delay) {
      // wait
    }
  }
protected:
  static void on_interrupt() { time_ = time_ + time_step.count(); }
};

static_assert(std::chrono::is_clock_v<steady_clock>);

class high_resolution_clock {
public:
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<high_resolution_clock, duration>;
  static constexpr bool is_steady = true;
private:
  static inline bool initialized_{false};
  static inline float nsec_per_tick_{0.0f};
public:
  high_resolution_clock() = delete;
  static void init();

  static bool initialized() { return initialized_; }

  static std::chrono::time_point<high_resolution_clock> now() {
    critical_section cs;

    duration const since_epoch{steady_clock::now().time_since_epoch()};

    // intermediate cast to int32 instead of immediate cast to rep (int64)
    // to use FPU and avoid usage of __fixsfdi
    int32_t nsec_count{
        static_cast<int32_t>(
            static_cast<float>(SysTick->LOAD - SysTick->VAL) * nsec_per_tick_)};

    duration const nsec{static_cast<rep>(nsec_count)};

    return time_point{since_epoch + nsec};
  }

  static void delay(std::chrono::nanoseconds delay) {
    auto const start{now()};
    while ((now() - start) <= delay) {
      // wait
    }
  }
};

static_assert(std::chrono::is_clock_v<high_resolution_clock>);

} // namespace chrono
} // namespace apm32
} // namespace mcu

#endif
#endif
