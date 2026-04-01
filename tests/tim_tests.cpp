#include <apm32/f4/tim.hpp>

using namespace apm32::f4::tim;

namespace {

static_assert(
    detail::calculate_prescaler<tim1>(
        emb::units::hz_f32{168000000},
        emb::units::hz_f32{10000},
        counter_mode::updown
    )
    == 0
);

static_assert(
    detail::calculate_prescaler<tim1>(
        emb::units::hz_f32{168000000},
        emb::units::hz_f32{1000},
        counter_mode::updown
    )
    == 1
);

static_assert(
    pwm::detail::get_deadtime_setup(
        emb::units::hz_f32{168000000},
        std::chrono::duration<int32_t, std::nano>{500},
        clock_division::div1
    )
    == 84
);

} // namespace
