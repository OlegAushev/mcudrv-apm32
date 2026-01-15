#pragma once

#include <cstdint>

namespace apm32 {
namespace f4 {
namespace adc {
namespace v2 {

enum class trigger_edge : uint8_t {
  rising = 0b01,
  falling = 0b10,
  both = 0b11
};

enum class inj_trigger_event : uint8_t {
  tim1_cc4 = 0b0000u,
  tim1_trgo = 0b0001u,
  tim2_cc1 = 0b0010u,
  tim2_trgo = 0b0011u,
  tim3_cc2 = 0b0100u,
  tim3_cc4 = 0b0101u,
  tim4_cc1 = 0b0110u,
  tim4_cc2 = 0b0111u,
  tim4_cc3 = 0b1000u,
  tim4_trgo = 0b1001u,
  tim5_cc4 = 0b1010u,
  tim5_trgo = 0b1011u,
  tim8_cc2 = 0b1100u,
  tim8_cc3 = 0b1101u,
  tim8_cc4 = 0b1110u,
  exti_line15 = 0b1111u
};

enum class reg_trigger_event : uint8_t {
  tim1_cc1 = 0b0000u,
  tim1_cc2 = 0b0001u,
  tim1_cc3 = 0b0010u,
  tim2_cc2 = 0b0011u,
  tim2_cc3 = 0b0100u,
  tim2_cc4 = 0b0101u,
  tim2_trgo = 0b0110u,
  tim3_cc1 = 0b0111u,
  tim3_trgo = 0b1000u,
  tim4_cc4 = 0b1001u,
  tim5_cc1 = 0b1010u,
  tim5_cc2 = 0b1011u,
  tim5_cc3 = 0b1100u,
  tim8_cc1 = 0b1101u,
  tim8_trgo = 0b1110u,
  exti_line11 = 0b1111u
};

struct inj_trigger {
  trigger_edge edge;
  inj_trigger_event event;
};

struct reg_trigger {
  trigger_edge edge;
  reg_trigger_event event;
};

enum class sampletime {
  cycles_3,
  cycles_15,
  cycles_28,
  cycles_56,
  cycles_84,
  cycles_112,
  cycles_144,
  cycles_480
};

} // namespace v2
} // namespace adc
} // namespace f4
} // namespace apm32
