#include <apm32/f4/can/can.hpp>

#include <apm32/f4/can/transceiver.hpp>

#include <cstdint>

using namespace apm32::f4::can;

namespace {

// 42 MHz: 14 time quanta per bit is the coarsest quantum whose sample point
// (85.7%) is still within tolerance of 87.5%.
static_assert(detail::find_bit_timing(42'000'000, 1'000'000, 0.875f)
              == bit_timing{.prescaler = 3,
                            .sync_jump_width = 1,
                            .time_segment1 = 11,
                            .time_segment2 = 2});

static_assert(detail::find_bit_timing(42'000'000, 500'000, 0.875f)
              == bit_timing{.prescaler = 6,
                            .sync_jump_width = 1,
                            .time_segment1 = 11,
                            .time_segment2 = 2});

// 8 time quanta would hit 87.5% exactly here (prescaler 21, segments 6 and 1),
// and is still rejected: the tolerance buys back resynchronization resolution.
static_assert(detail::find_bit_timing(42'000'000, 250'000, 0.875f)
              == bit_timing{.prescaler = 12,
                            .sync_jump_width = 1,
                            .time_segment1 = 11,
                            .time_segment2 = 2});

static_assert(detail::find_bit_timing(42'000'000, 125'000, 0.875f)
              == bit_timing{.prescaler = 21,
                            .sync_jump_width = 1,
                            .time_segment1 = 13,
                            .time_segment2 = 2});

static_assert(detail::find_bit_timing(36'000'000, 500'000, 0.875f)
              == bit_timing{.prescaler = 4,
                            .sync_jump_width = 1,
                            .time_segment1 = 15,
                            .time_segment2 = 2});

// 42 MHz is not an integer multiple of any 800 kbit/s bit length.
static_assert(!detail::find_bit_timing(42'000'000, 800'000, 0.875f));

// A wider jump width is the caller's to ask for, up to min(4, TS2); wider than
// that does not compile.
static_assert(
    detail::with_sync_jump_width(
        *detail::find_bit_timing(42'000'000, 500'000, 0.875f), 2)
    == bit_timing{.prescaler = 6,
                  .sync_jump_width = 2,
                  .time_segment1 = 11,
                  .time_segment2 = 2});

static_assert(bit_timing{.prescaler = 6,
                         .sync_jump_width = 1,
                         .time_segment1 = 11,
                         .time_segment2 = 2}
                  .reg_bits()
              == 0x001A0005u);

} // namespace
