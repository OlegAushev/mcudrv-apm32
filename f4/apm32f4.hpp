#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <emb/core.hpp>
#include <mcudrv-apm32/apm32.hpp>

namespace mcu {

enum class exec_status {
  ok,
  error,
  busy,
  timeout,
  invalid_argument,
  overflow,
};

inline void fatal_error() {
  emb::fatal_error("mcudrv fatal error");
}

inline void fatal_error(char const* hint, int code = 0) {
  emb::fatal_error(hint, code);
}

} // namespace mcu

#endif
#endif
