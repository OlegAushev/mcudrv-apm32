#pragma once

#ifdef APM32F4XX

#include <apm32f4xx_crc.h>
#include <apm32f4xx_rcm.h>

#include <mcu/apm32/f4/system.hpp>

#include <cstddef>

namespace mcu {
inline namespace apm32 {
inline namespace f4 {
namespace crc {

inline void init() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_CRC); }

inline uint32_t calc_crc32(const uint8_t* buf, size_t len) {
    CRC->CTRL_B.RST = BIT_SET;
    for (auto i = 0uz; i < len; ++i) {
        CRC->DATA = buf[i];
    }
    return CRC->DATA;
}

} // namespace crc
} // namespace f4
} // namespace apm32
} // namespace mcu

#endif
