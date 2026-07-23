#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <utility>

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

inline constexpr std::uintptr_t base = 0x0800'0000;

#if defined(APM32F4_FLASH_512K)
inline constexpr std::size_t capacity = 0x8'0000;
inline constexpr std::uint32_t sector_count = 8;
#else
inline constexpr std::size_t capacity = 0x10'0000;
inline constexpr std::uint32_t sector_count = 12;
#endif

constexpr auto address(sector s) -> std::uintptr_t {
  auto const n = std::to_underlying(s);
  return n <= 3 ? base + 0x4000 * n
       : n == 4 ? base + 0x1'0000
                : base + 0x2'0000 * (n - 4);
}

constexpr auto size(sector s) -> std::size_t {
  auto const n = std::to_underlying(s);
  return n <= 3 ? 0x4000 : n == 4 ? 0x1'0000 : 0x2'0000;
}

inline auto view(sector s) -> std::span<std::byte const> {
  return {reinterpret_cast<std::byte const*>(address(s)), size(s)};
}

static_assert(address(sector::_0) == base);
static_assert(address(sector::_4) == base + 0x1'0000);
static_assert(address(sector::_5) == base + 0x2'0000);
static_assert([] {
  for (std::uint32_t n = 0; n + 1 < sector_count; ++n) {
    if (address(sector{n}) + size(sector{n}) != address(sector{n + 1})) {
      return false;
    }
  }
  auto const last = sector{sector_count - 1};
  return address(last) + size(last) == base + capacity;
}());

// Factory-programmed flash size of the actual part; may differ from capacity.
inline auto device_flash_size() -> std::size_t {
  return *reinterpret_cast<std::uint16_t volatile*>(FLASHSIZE_BASE) * 1024u;
}

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

constexpr auto erased(std::span<std::byte const> data) -> bool {
  for (std::byte b : data) {
    if (b != std::byte{0xFF}) {
      return false;
    }
  }
  return true;
}

auto erase_sector(sector s) -> std::expected<void, error>;

// Programming only clears bits and the hardware does not detect overwrites:
// the target range must be erased beforehand.
auto write(std::uintptr_t addr, std::span<std::byte const> data)
    -> std::expected<void, error>;

auto write_byte(std::uintptr_t addr, std::byte b) -> std::expected<void, error>;

} // namespace apm32::f4::flash
