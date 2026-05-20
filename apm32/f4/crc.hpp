#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <cstddef>

namespace apm32 {
namespace f4 {
namespace crc {

inline void init() { emb::mmio::set(RCM->AHB1CLKEN, RCM_AHB1CLKEN_CRCEN); }

inline uint32_t calc_crc32(const uint8_t* buf, size_t len) {
    emb::mmio::set(CRC->CTRL, CRC_CTRL_RST);
    for (auto i = 0uz; i < len; ++i) {
        CRC->DATA = buf[i];
    }
    return CRC->DATA;
}

} // namespace crc
} // namespace f4
} // namespace apm32
