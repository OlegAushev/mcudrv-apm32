#pragma once

#include <apm32/f4/can/can_instances.hpp>

namespace apm32 {
namespace f4 {
namespace can {

struct filter_init_session {
  filter_init_session() {
    emb::mmio::set(can1::REG.FCTRL, CAN_FCTRL_FINITEN);
  }

  ~filter_init_session() {
    emb::mmio::clear(can1::REG.FCTRL, CAN_FCTRL_FINITEN);
  }
};

} // namespace can
} // namespace f4
} // namespace apm32
