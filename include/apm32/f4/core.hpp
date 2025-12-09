#pragma once

#include <apm32/device.hpp>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include <emb/units.hpp>

namespace apm32 {
namespace f4 {
namespace core {

void init_clock();
void update_clock();
void init_core();
void reset_device();
[[noreturn]] void halt_device();

template<typename T>
inline T clock_frequency() {
  if constexpr (requires { typename T::underlying_type; }) {
    return T{static_cast<typename T::underlying_type>(SystemCoreClock)};
  } else {
    return static_cast<T>(SystemCoreClock);
  }
}

template<typename T>
inline T apb1_timer_frequency() {
  return clock_frequency<T>() / 2;
}

template<typename T>
inline T apb2_timer_frequency() {
  return clock_frequency<T>();
}

inline uint32_t serial_number() {
  uint32_t* uid_ptr{reinterpret_cast<uint32_t*>(0x1FFF7A10)};
  return *uid_ptr;
}

constexpr void ensure(bool pred) {
  if !consteval {
    if (!pred) {
      halt_device();
    }
  } else {
    assert(pred);
  }
}

template<typename Derived, typename Id, size_t Count>
class peripheral {
private:
  static inline std::array<Derived*, Count> instances_{};
protected:
  peripheral(Id id) {
    auto idx = static_cast<size_t>(id);
    ensure(idx < Count);
    ensure(!initialized(id));
    instances_[idx] = static_cast<Derived*>(this);
  }
public:
  peripheral(peripheral const&) = delete;
  peripheral& operator=(peripheral const&) = delete;
  peripheral(peripheral&&) = delete;
  peripheral& operator=(peripheral&&) = delete;

  static Derived* instance(Id id) {
    auto idx = static_cast<size_t>(id);
    ensure(idx < Count);
    ensure(initialized(id));
    return instances_[idx];
  }

  static bool initialized(Id id) {
    auto idx = static_cast<size_t>(id);
    ensure(idx < Count);
    return instances_[idx] != nullptr;
  }
};

} // namespace core
} // namespace f4
} // namespace apm32
