#pragma once

#include <apm32/device.hpp>

#include <array>
#include <cstdint>

namespace apm32::f4::core {

void init_core();

void reset_device();

[[noreturn]] void halt_device();

// Factory-programmed 96-bit unique ID of the part, U_ID[31:0] first.
inline std::array<std::uint32_t, 3> uid()
{
  auto const* words = reinterpret_cast<std::uint32_t const volatile*>(UID_BASE);
  return {words[0], words[1], words[2]};
}

} // namespace apm32::f4::core
