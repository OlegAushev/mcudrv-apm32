#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <utility>

namespace apm32 {
namespace f4 {
namespace exti {

enum class line : uint32_t {
    line0  = (1u << 0),
    line1  = (1u << 1),
    line2  = (1u << 2),
    line3  = (1u << 3),
    line4  = (1u << 4),
    line5  = (1u << 5),
    line6  = (1u << 6),
    line7  = (1u << 7),
    line8  = (1u << 8),
    line9  = (1u << 9),
    line10 = (1u << 10),
    line11 = (1u << 11),
    line12 = (1u << 12),
    line13 = (1u << 13),
    line14 = (1u << 14),
    line15 = (1u << 15),
    line16 = (1u << 16),
    line17 = (1u << 17),
    line18 = (1u << 18),
    line19 = (1u << 19),
    line20 = (1u << 20),
    line21 = (1u << 21),
    line22 = (1u << 22)
};

enum class mode {
  interrupt,
  event
};

enum class trigger_edge {
  rising,
  falling,
  both
};

struct Config {
  exti::mode mode;
  exti::trigger_edge trigger_edge;
};

void configure(line l, mode m, trigger_edge edge);

inline void trigger_software_interrupt(line l) {
  emb::mmio::set(EINT->SWINTE, std::to_underlying(l));
}

inline void ack_interrupt(line l) {
  // IPEND is rc_w1: writing 1 clears the pending bit
  emb::mmio::clear_w1(EINT->IPEND, std::to_underlying(l));
}

} // namespace exti
} // namespace f4
} // namespace apm32
