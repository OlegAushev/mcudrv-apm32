#include <apm32/f4/spi/spi_utils.hpp>

using namespace apm32::f4::spi;

namespace {

static_assert(
    detail::calculate_prescaler(
        emb::units::hz_f32{84000000},
        emb::units::hz_f32{20000000}
    )
    == baudrate_prescaler::div8
);

static_assert(
    detail::calculate_prescaler(
        emb::units::hz_f32{42000000},
        emb::units::hz_f32{1000000}
    )
    == baudrate_prescaler::div64
);

} // namespace
