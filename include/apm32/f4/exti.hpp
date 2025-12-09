#pragma once

#include <apm32/device.hpp>
#include <apm32/utility.hpp>

#include <apm32f4xx_eint.h>

#include <utility>

namespace apm32 {
namespace f4 {
namespace exti {

enum class line : uint32_t {
    line0 = EINT_LINE_0,
    line1 = EINT_LINE_1,
    line2 = EINT_LINE_2,
    line3 = EINT_LINE_3,
    line4 = EINT_LINE_4,
    line5 = EINT_LINE_5,
    line6 = EINT_LINE_6,
    line7 = EINT_LINE_7,
    line8 = EINT_LINE_8,
    line9 = EINT_LINE_9,
    line10 = EINT_LINE_10,
    line11 = EINT_LINE_11,
    line12 = EINT_LINE_12,
    line13 = EINT_LINE_13,
    line14 = EINT_LINE_14,
    line15 = EINT_LINE_15,
    line16 = EINT_LINE_16,
    line17 = EINT_LINE_17,
    line18 = EINT_LINE_18,
    line19 = EINT_LINE_19,
    line20 = EINT_LINE_20,
    line21 = EINT_LINE_21,
    line22 = EINT_LINE_22
};

enum class mode {
  interrupt = EINT_MODE_INTERRUPT,
  event = EINT_MODE_EVENT
};

enum class trigger_edge {
  rising = EINT_TRIGGER_RISING,
  falling = EINT_TRIGGER_FALLING,
  both = EINT_TRIGGER_RISING_FALLING
};

struct Config {
  exti::mode mode;
  exti:: trigger_edge trigger_edge;
};

void configure(line l, mode m, trigger_edge edge);

inline void trigger_software_interrupt(line l) {
  apm32::set_bit(EINT->SWINTE, std::to_underlying(l));
}

inline void ack_interrupt(line l) {
  apm32::set_bit(EINT->IPEND, std::to_underlying(l));
}

} // namespace exti
} // namespace f4
} // namespace apm32
