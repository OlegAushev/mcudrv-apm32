#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

namespace apm32::f4::flash {

enum class error {
  operation,
  write_protection,
  programming_alignment,
  programming_parallelism,
  programming_sequence,
  locked
};

enum class sector : std::uint32_t {
  _0,
  _1,
  _2,
  _3,
  _4,
  _5,
  _6,
  _7,
  _8,
  _9,
  _10,
  _11
};

enum class voltage_range : std::uint32_t {
  _1 = 0x00, // 1.8V to 2.1V, operation by byte (8-bit)
  _2 = 0x01, // 2.1V to 2.7V, operation by half word (16-bit)
  _3 = 0x02, // 2.7V to 3.6V, operation by word (32-bit)
  _4 = 0x03, // 2.7V to 3.6V + External Vpp, operation by double word (64-bit)
};

enum class program_size : std::uint32_t { _8, _16, _32, _64 };

constexpr std::uint32_t wait_states(std::uint64_t hclk_hz) {
  return hclk_hz <= 30'000'000  ? FLASH_ACCTRL_WAITP_0WS
       : hclk_hz <= 60'000'000  ? FLASH_ACCTRL_WAITP_1WS
       : hclk_hz <= 90'000'000  ? FLASH_ACCTRL_WAITP_2WS
       : hclk_hz <= 120'000'000 ? FLASH_ACCTRL_WAITP_3WS
       : hclk_hz <= 150'000'000 ? FLASH_ACCTRL_WAITP_4WS
                                : FLASH_ACCTRL_WAITP_5WS;
}

inline void enable_acceleration() {
  emb::mmio::set<
      FLASH_ACCTRL_PREFEN | FLASH_ACCTRL_ICACHEEN | FLASH_ACCTRL_DCACHEEN>(
      FLASH->ACCTRL
  );
}

auto erase_sector(sector s) -> std::expected<void, error>;

auto write(std::uintptr_t addr, std::span<std::byte const> data)
    -> std::expected<void, error>;

auto write_byte(std::uintptr_t addr, std::byte b) -> std::expected<void, error>;

} // namespace apm32::f4::flash
