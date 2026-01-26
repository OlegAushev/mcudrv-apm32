#pragma once

#include <apm32/device.hpp>

#include <apm32/f4/nvic.hpp>

#include <emb/seqlock.hpp>

#include <chrono>

extern "C" void SysTick_Handler();

namespace apm32 {
namespace f4 {
namespace chrono {

class high_resolution_clock;

class steady_clock {
  friend void ::SysTick_Handler();
  friend class high_resolution_clock;
public:
  using duration = std::chrono::milliseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<steady_clock, duration>;
  static constexpr bool is_steady = true;
private:
  static inline bool initialized_ = false;
  static inline emb::seqlock<int64_t> time_{};
public:
  steady_clock() = delete;
  static void init();

  static bool initialized() {
    return initialized_;
  }

  static std::chrono::time_point<steady_clock> now() {
    return time_point{std::chrono::milliseconds{time_.load()}};
  }

  static void delay(std::chrono::milliseconds delay) {
    auto const start = now();
    while ((now() - start) <= delay) {
      // wait
    }
  }
protected:
  static void on_interrupt() {
    time_.update([](int64_t t) { return ++t; });
  }
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
  static inline bool initialized_ = false;
  static inline float nsec_per_tick_ = 0.0f;
public:
  high_resolution_clock() = delete;
  static void init();

  static bool initialized() {
    return initialized_;
  }

  static std::chrono::time_point<high_resolution_clock> now() {
#if 1
    int64_t ms;
    uint32_t ticks;

    // Read milliseconds and SysTick value until we get a consistent pair.
    // If SysTick interrupt fires between the two reads of time_, the values
    // won't match and we retry. This handles calls from normal code or from
    // ISRs with priority lower than SysTick.
    do {
      ms = steady_clock::time_.load();
      ticks = SysTick->LOAD - SysTick->VAL;
    } while (ms != steady_clock::time_.load());

    // Handle calls from ISRs with same or higher priority as SysTick.
    // In this case, SysTick cannot preempt us, so the loop above always
    // exits immediately. However, SysTick may have already fired and be
    // pending — VAL has reset to LOAD but time_ hasn't been incremented yet.
    // The pending flag tells us this happened, so we manually adjust.
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
      ++ms;
      // Re-read ticks since VAL is now counting down from LOAD in the new
      // millisecond period, making our previous ticks reading stale.
      ticks = SysTick->LOAD - SysTick->VAL;
    }

    // Convert ticks to nanoseconds using FPU. Intermediate cast to int32_t
    // instead of direct cast to int64_t allows use of hardware FPU instruction
    // and avoids software floating-point conversion routine __fixsfdi
    // that would be needed for float-to-int64 conversion.
    auto const nsec_count = static_cast<int32_t>(
        static_cast<float>(ticks) * nsec_per_tick_
    );

    return time_point{
        std::chrono::duration_cast<duration>(std::chrono::milliseconds{ms}) +
        duration{nsec_count}
    };
#else
    // TODO remove this after testing
    nvic::irq_guard lock;

    auto const since_epoch = steady_clock::now().time_since_epoch();

    // intermediate cast to int32 instead of immediate cast to rep (int64)
    // to use FPU and avoid usage of __fixsfdi
    auto const nsec_count = static_cast<int32_t>(
        static_cast<float>(SysTick->LOAD - SysTick->VAL) * nsec_per_tick_
    );

    auto const nsec = duration{static_cast<rep>(nsec_count)};

    return time_point(since_epoch + nsec);
#endif
  }

  static void delay(std::chrono::nanoseconds delay) {
    auto const start = now();
    while ((now() - start) <= delay) {
      // wait
    }
  }
};

static_assert(std::chrono::is_clock_v<high_resolution_clock>);

constexpr float to_float(high_resolution_clock::duration dur) {
  // intermediate cast to int32 instead of immediate cast to float to use FPU
  int32_t const dur_ = static_cast<int32_t>(dur.count());
  return static_cast<float>(dur_);
}

} // namespace chrono
} // namespace f4
} // namespace apm32
