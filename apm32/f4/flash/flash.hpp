#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <cstdint>
#include <expected>
#include <optional>

namespace apm32::f4::flash {

enum class error {
  operation,
  write_protection,
  programming_alignment,
  programming_parallelism,
  programming_sequence,
};

enum class sector : std::uint16_t {
  _0 = 0x0000,
  _1 = 0x0008,
  _2 = 0x0010,
  _3 = 0x0018,
  _4 = 0x0020,
  _5 = 0x0028,
  _6 = 0x0030,
  _7 = 0x0038,
  _8 = 0x0040,
  _9 = 0x0048,
  _10 = 0x0050,
  _11 = 0x0058,
  _12 = 0x0080,
  _13 = 0x0088,
  _14 = 0x0090,
  _15 = 0x0098,
  _16 = 0x00A0,
  _17 = 0x00A8,
  _18 = 0x00B0,
  _19 = 0x00B8,
  _20 = 0x00C0,
  _21 = 0x00C8,
  _22 = 0x00D0,
  _23 = 0x00D8
};

enum class voltage_range : std::uint8_t {
  _1 = 0x00, // 1.8V to 2.1V, operation by byte (8-bit)
  _2 = 0x01, // 2.1V to 2.7V, operation by half word (16-bit)
  _3 = 0x02, // 2.7V to 3.6V, operation by word (32-bit)
  _4 = 0x03, // 2.7V to 3.6V + External Vpp, operation by double word (64-bit)
};

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

inline void unlock() {
  constexpr std::uint32_t key1 = 0x45670123;
  constexpr std::uint32_t key2 = 0xCDEF89AB;
  if (emb::mmio::test<FLASH_CTRL_LOCK>(FLASH->CTRL)) {
    FLASH->KEY = key1;
    FLASH->KEY = key2;
  }
}

inline void lock(void) {
  emb::mmio::set<FLASH_CTRL_LOCK>(FLASH->CTRL);
}

inline bool busy() {
  return emb::mmio::test<FLASH_STS_BUSY>(FLASH->STS);
}

inline std::optional<error> error_status() {
  constexpr auto errors = FLASH_STS_WPROTERR
                        | FLASH_STS_PGALGERR
                        | FLASH_STS_PGPRLERR
                        | FLASH_STS_PGSEQERR
                        | FLASH_STS_OPRERR;

  if (!emb::mmio::test_any<errors>(FLASH->STS)) {
    return {};
  }

  if (emb::mmio::test<FLASH_STS_WPROTERR>(FLASH->STS)) {
    return error::write_protection;
  } else if (emb::mmio::test<FLASH_STS_PGALGERR>(FLASH->STS)) {
    return error::programming_alignment;
  } else if (emb::mmio::test<FLASH_STS_PGPRLERR>(FLASH->STS)) {
    return error::programming_parallelism;
  } else if (emb::mmio::test<FLASH_STS_PGSEQERR>(FLASH->STS)) {
    return error::programming_sequence;
  } else if (emb::mmio::test<FLASH_STS_OPRERR>(FLASH->STS)) {
    return error::operation;
  }

  return {};
}

inline auto erase_sector(sector s) -> std::expected<void, error> {
  // TODO
  return {};
}

inline auto write_byte(std::byte b) -> std::expected<void, error> {
  // TODO
  return {};
}

} // namespace apm32::f4::flash
