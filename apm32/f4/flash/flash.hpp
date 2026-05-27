#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <cstdint>

namespace apm32::f4::flash {

constexpr std::uint32_t wait_states(std::uint64_t hclk_hz) {
  return hclk_hz <= 30'000'000  ? FLASH_ACCTRL_WAITP_0WS
       : hclk_hz <= 60'000'000  ? FLASH_ACCTRL_WAITP_1WS
       : hclk_hz <= 90'000'000  ? FLASH_ACCTRL_WAITP_2WS
       : hclk_hz <= 120'000'000 ? FLASH_ACCTRL_WAITP_3WS
       : hclk_hz <= 150'000'000 ? FLASH_ACCTRL_WAITP_4WS
                                : FLASH_ACCTRL_WAITP_5WS;
}

inline void enable_acceleration() {
  emb::mmio::set(
      FLASH->ACCTRL,
      FLASH_ACCTRL_PREFEN | FLASH_ACCTRL_ICACHEEN | FLASH_ACCTRL_DCACHEEN
  );
}

} // namespace apm32::f4::flash
