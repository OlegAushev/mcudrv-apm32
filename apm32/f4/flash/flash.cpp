#include <apm32/f4/flash/flash.hpp>

#include <emb/expected.hpp>
#include <emb/scope.hpp>

namespace apm32::f4::flash {

namespace {

auto locked() -> bool {
  return emb::mmio::test<FLASH_CTRL_LOCK>(FLASH->CTRL);
}

auto unlock() -> std::expected<void, error> {
  constexpr std::uint32_t key1 = 0x45670123;
  constexpr std::uint32_t key2 = 0xCDEF89AB;

  if (!locked()) {
    return {};
  }

  FLASH->KEY = key1;
  FLASH->KEY = key2;

  if (locked()) {
    // A wrong key sequence locks CTRL until reset.
    // Do not retry unlock after this.
    return std::unexpected(error::locked);
  }

  return {};
}

auto lock() -> void {
  emb::mmio::set<FLASH_CTRL_LOCK>(FLASH->CTRL);
}

auto busy() -> bool {
  return emb::mmio::test<FLASH_STS_BUSY>(FLASH->STS);
}

constexpr auto sts_errors = FLASH_STS_WPROTERR
                          | FLASH_STS_PGALGERR
                          | FLASH_STS_PGPRLERR
                          | FLASH_STS_PGSEQERR
                          | FLASH_STS_OPRERR;

auto clear_errors() -> void {
  emb::mmio::clear_w1<sts_errors>(FLASH->STS);
}

auto check_errors() -> std::expected<void, error> {
  std::uint32_t const sts = FLASH->STS;

  if (!emb::mmio::test_any<sts_errors>(sts)) {
    return {};
  }

  if (emb::mmio::test<FLASH_STS_WPROTERR>(sts)) {
    return std::unexpected(error::write_protection);
  } else if (emb::mmio::test<FLASH_STS_PGALGERR>(sts)) {
    return std::unexpected(error::programming_alignment);
  } else if (emb::mmio::test<FLASH_STS_PGPRLERR>(sts)) {
    return std::unexpected(error::programming_parallelism);
  } else if (emb::mmio::test<FLASH_STS_PGSEQERR>(sts)) {
    return std::unexpected(error::programming_sequence);
  } else {
    return std::unexpected(error::operation);
  }
}

// Programming keeps the data cache coherent, erasing does not: after a
// sector erase the caches may still hold its old contents, so flush both.
// The reset bits may only be set while the corresponding cache is disabled.
auto reset_caches() -> void {
  std::uint32_t const acctrl = FLASH->ACCTRL;

  emb::mmio::clear<FLASH_ACCTRL_ICACHEEN | FLASH_ACCTRL_DCACHEEN>(
      FLASH->ACCTRL
  );

  emb::mmio::set<FLASH_ACCTRL_ICACHERST | FLASH_ACCTRL_DCACHERST>(
      FLASH->ACCTRL
  );

  emb::mmio::clear<FLASH_ACCTRL_ICACHERST | FLASH_ACCTRL_DCACHERST>(
      FLASH->ACCTRL
  );

  FLASH->ACCTRL = acctrl;
}

} // namespace

auto erase_sector(sector s) -> std::expected<void, error> {
  TRY(unlock());
  auto relock = emb::scope_exit([] { lock(); });

  while (busy()) {}
  clear_errors();

  emb::mmio::modify(
      FLASH->CTRL,
      emb::mmio::bits<FLASH_CTRL_PGSIZE>(program_size::_8),
      emb::mmio::bits<FLASH_CTRL_SERS>(1u),
      emb::mmio::bits<FLASH_CTRL_SNUM>(s)
  );
  emb::mmio::set<FLASH_CTRL_START>(FLASH->CTRL);

  while (busy()) {}
  emb::mmio::clear<FLASH_CTRL_SERS>(FLASH->CTRL);

  reset_caches();

  return check_errors();
}

auto write(std::uintptr_t addr, std::span<std::byte const> data)
    -> std::expected<void, error> {
  TRY(unlock());
  auto relock = emb::scope_exit([] { lock(); });

  while (busy()) {}
  clear_errors();

  emb::mmio::write<FLASH_CTRL_PGSIZE>(FLASH->CTRL, program_size::_8);
  emb::mmio::set<FLASH_CTRL_PG>(FLASH->CTRL);
  auto unprogram = emb::scope_exit([] {
    emb::mmio::clear<FLASH_CTRL_PG>(FLASH->CTRL);
  });

  auto* dst = reinterpret_cast<std::byte volatile*>(addr);
  for (std::byte b : data) {
    *dst++ = b;
    // Drain the write buffer so BUSY is observed set before the first poll.
    __DSB();
    while (busy()) {}
    TRY(check_errors());
  }

  return {};
}

auto write_byte(std::uintptr_t addr, std::byte b)
    -> std::expected<void, error> {
  return write(addr, {&b, 1});
}

} // namespace apm32::f4::flash
