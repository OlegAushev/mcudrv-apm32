#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx.h>
#include <apm32f4xx_eint.h>
#include <apm32f4xx_rcm.h>

#include <mcu/apm32/f4/system.hpp>

namespace mcu {
inline namespace apm32 {
namespace exti{

enum class Line : uint32_t {
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

enum class Mode {
  interrupt = EINT_MODE_INTERRUPT,
  event = EINT_MODE_EVENT
};

enum class TriggerEdge {
  rising = EINT_TRIGGER_RISING,
  falling = EINT_TRIGGER_FALLING,
  both = EINT_TRIGGER_RISING_FALLING
};

struct Config {
  Mode mode;
  TriggerEdge trigger_edge;
};

void init(Line line, Mode mode, TriggerEdge edge);

inline void trigger_software_interrupt(Line line) {
  mcu::set_bit(EINT->SWINTE, std::to_underlying(line));
}

inline void ack_interrupt(Line line) {
  mcu::set_bit(EINT->IPEND, std::to_underlying(line));
}

} // namespace exti
} // namespace apm32
} // namespace mcu

#endif
#endif
