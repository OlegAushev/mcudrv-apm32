#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <apm32/f4/flash/flash.hpp>
#include <apm32/f4/rcc/rcc_types.hpp>
#include <rcc_config.hpp>

#ifndef HSE_VALUE
#error "HSE_VALUE not defined"
#endif

namespace apm32::f4::rcc {

inline constexpr std::uint64_t hse_value = HSE_VALUE;
inline constexpr std::uint64_t hsi_value = 16'000'000;

inline constexpr std::uint64_t hse_min = 4'000'000;
inline constexpr std::uint64_t hse_max = 26'000'000;
inline constexpr std::uint64_t vco_in_min = 1'000'000;
inline constexpr std::uint64_t vco_in_max = 2'000'000;
inline constexpr std::uint64_t vco_out_min = 100'000'000;
inline constexpr std::uint64_t vco_out_max = 432'000'000;
inline constexpr std::uint64_t sysclk_max = 168'000'000;
inline constexpr std::uint64_t hclk_max = 168'000'000;
inline constexpr std::uint64_t pclk1_max = 42'000'000;
inline constexpr std::uint64_t pclk2_max = 84'000'000;

template<typename T>
constexpr T hse_frequency() {
  return T(hse_value);
}

template<typename T>
constexpr T hsi_frequency() {
  return T(hsi_value);
}

template<typename T>
constexpr T sysclk_frequency() {
  using C = clock_config;
  if constexpr (C::sysclk_src == sysclk_src::hse) {
    return hse_frequency<T>();
  } else if constexpr (C::sysclk_src == sysclk_src::hsi) {
    return hsi_frequency<T>();
  } else {
    constexpr uint64_t pll_in = C::pll_src == pll_src::hse ? hse_value
                                                           : hsi_value;

    static_assert(
        pll_in % C::pllb_div == 0,
        "PLL VCO input frequency is not an integer number of Hz"
    );
    constexpr uint64_t vco_in = pll_in / C::pllb_div;
    static_assert(
        vco_in >= vco_in_min && vco_in <= vco_in_max,
        "PLL VCO input frequency out of range"
    );

    constexpr uint64_t vco_out = vco_in * C::pll1a_mult;
    static_assert(
        vco_out >= vco_out_min && vco_out <= vco_out_max,
        "PLL VCO output frequency out of range"
    );

    static_assert(
        vco_out % to_divisor(C::pll1c_div) == 0,
        "PLL output frequency is not an integer number of Hz"
    );
    constexpr uint64_t pll_out = vco_out / to_divisor(C::pll1c_div);

    return T(pll_out);
  }
}

template<typename T>
constexpr T hclk_frequency() {
  using C = clock_config;
  return sysclk_frequency<T>() / to_divisor(C::ahb_div);
}

template<typename T>
constexpr T cpu_frequency() {
  return hclk_frequency<T>();
}

template<typename T>
constexpr T pclk1_frequency() {
  using C = clock_config;
  return hclk_frequency<T>() / to_divisor(C::apb1_div);
}

template<typename T>
constexpr T pclk1_timer_frequency() {
  using C = clock_config;
  return C::apb1_div == apb_div::_1 ? pclk1_frequency<T>()
                                    : pclk1_frequency<T>() * 2;
}

template<typename T>
constexpr T pclk2_frequency() {
  using C = clock_config;
  return hclk_frequency<T>() / to_divisor(C::apb2_div);
}

template<typename T>
constexpr T pclk2_timer_frequency() {
  using C = clock_config;
  return C::apb2_div == apb_div::_1 ? pclk2_frequency<T>()
                                    : pclk2_frequency<T>() * 2;
}

inline void init_clock() {
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

// ---- Clock Config Validation ----

static_assert(
    clock_config::pllb_div >= 2 && clock_config::pllb_div <= 63,
    "pllb_div must be in 2..63"
);

static_assert(
    clock_config::pll1a_mult >= 50 && clock_config::pll1a_mult <= 432,
    "pll1a_mult must be in 50..432"
);

static_assert(
    clock_config::plld_div >= 2 && clock_config::plld_div <= 15,
    "plld_div must be in 2..15"
);

static_assert(
    hse_value >= hse_min && hse_value <= hse_max,
    "HSE_VALUE out of supported range"
);

static_assert(
    sysclk_frequency<std::uint64_t>() % to_divisor(clock_config::ahb_div) == 0,
    "HCLK frequency is not an integer number of Hz"
);

static_assert(
    hclk_frequency<std::uint64_t>() % to_divisor(clock_config::apb1_div) == 0,
    "PCLK1 frequency is not an integer number of Hz"
);

static_assert(
    hclk_frequency<std::uint64_t>() % to_divisor(clock_config::apb2_div) == 0,
    "PCLK2 frequency is not an integer number of Hz"
);

static_assert(
    sysclk_frequency<std::uint64_t>() <= sysclk_max,
    "SYSCLK frequency exceeds maximum"
);

static_assert(
    hclk_frequency<std::uint64_t>() <= hclk_max,
    "HCLK frequency exceeds maximum"
);

static_assert(
    cpu_frequency<decltype(clock_config::expected_cpu_frequency)>()
        == clock_config::expected_cpu_frequency,
    "computed CPU frequency does not match expected CPU frequency"
);

static_assert(
    pclk1_frequency<std::uint64_t>() <= pclk1_max,
    "PCLK1 frequency exceeds maximum"
);

static_assert(
    pclk2_frequency<std::uint64_t>() <= pclk2_max,
    "PCLK2 frequency exceeds maximum"
);

} // namespace apm32::f4::rcc
