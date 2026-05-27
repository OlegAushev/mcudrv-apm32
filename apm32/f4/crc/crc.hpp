#pragma once

#include <apm32/device.hpp>

#include <emb/mmio.hpp>

#include <cstddef>
#include <cstdint>

namespace apm32::f4::crc {

inline void init() { emb::mmio::set(RCM->AHB1CLKEN, RCM_AHB1CLKEN_CRCEN); }

inline std::uint32_t calc_crc32(const std::uint8_t* buf, std::size_t len) {
    emb::mmio::set(CRC->CTRL, CRC_CTRL_RST);
    for (auto i = 0uz; i < len; ++i) {
        CRC->DATA = buf[i];
    }
    return CRC->DATA;
}

} // namespace apm32::f4::crc
