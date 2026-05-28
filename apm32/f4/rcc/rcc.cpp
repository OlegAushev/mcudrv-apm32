#include <apm32/f4/rcc/rcc.hpp>

#include <apm32/f4/flash/flash.hpp>

#include <emb/mmio.hpp>

namespace apm32::f4::rcc {

void init_clock() {
  using C = clock_config;

  // Enable HSE if it is required as SYSCLK source or PLL input
  if constexpr (C::sysclk_src == sysclk_src::hse
                || (C::sysclk_src == sysclk_src::pll
                    && C::pll_src == pll_src::hse)) {
    emb::mmio::set(RCM->CTRL, RCM_CTRL_HSEEN);
    while (!emb::mmio::test_any(RCM->CTRL, RCM_CTRL_HSERDYFLG)) {}
  }
  // HSI is enabled by hardware after reset

  // Enable PMU clock and select voltage regulator scale 1
  // (required for SYSCLK > 144 MHz; safe at any frequency)
  emb::mmio::set(RCM->APB1CLKEN, RCM_APB1CLKEN_PMUEN);
  emb::mmio::set(PMU->CTRL, PMU_CTRL_VOSSEL);

  // Configure bus prescalers
  emb::mmio::write(
      RCM->CFG,
      RCM_CFG_AHBPSC,
      static_cast<std::uint32_t>(C::ahb_div)
  );
  emb::mmio::write(
      RCM->CFG,
      RCM_CFG_APB2PSC,
      static_cast<std::uint32_t>(C::apb2_div)
  );
  emb::mmio::write(
      RCM->CFG,
      RCM_CFG_APB1PSC,
      static_cast<std::uint32_t>(C::apb1_div)
  );

  // Configure and enable PLL when selected as SYSCLK source
  if constexpr (C::sysclk_src == sysclk_src::pll) {
    emb::mmio::modify(
        RCM->PLL1CFG,
        emb::mmio::bits<RCM_PLL1CFG_PLLB>{C::pllb_div},
        emb::mmio::bits<RCM_PLL1CFG_PLL1A>{C::pll1a_mult},
        emb::mmio::bits<RCM_PLL1CFG_PLL1C>{
            static_cast<std::uint32_t>(C::pll1c_div)
        },
        emb::mmio::bits<RCM_PLL1CFG_PLLD>{C::plld_div},
        emb::mmio::bits<RCM_PLL1CFG_PLL1CLKS>{
            static_cast<std::uint32_t>(C::pll_src)
        }
    );
    emb::mmio::set(RCM->CTRL, RCM_CTRL_PLL1EN);
    while (!emb::mmio::test_any(RCM->CTRL, RCM_CTRL_PLL1RDYFLG)) {}
  }

  // Set Flash wait states based on the target HCLK frequency
  emb::mmio::write(
      FLASH->ACCTRL,
      FLASH_ACCTRL_WAITP,
      flash::wait_states(hclk_frequency<std::uint64_t>())
  );

  // Switch SYSCLK to the configured source
  constexpr auto sclk_field = static_cast<std::uint32_t>(C::sysclk_src);
  emb::mmio::write(RCM->CFG, RCM_CFG_SCLKSEL, sclk_field);
  while (emb::mmio::read(RCM->CFG, RCM_CFG_SCLKSELSTS) != sclk_field) {}

  // Sync the runtime SystemCoreClock variable from configured registers
  SystemCoreClockUpdate();
}

} // namespace apm32::f4::rcc
