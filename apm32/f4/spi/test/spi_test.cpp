#include <apm32/f4/spi/spi.hpp>

#include <apm32/f4/spi/blocking_master.hpp>

#include <concepts>
#include <cstdint>

using namespace apm32::f4::spi;

namespace {

// A single-slave master is itself what a portable device driver takes: the
// one chip select it owns is the part's, so nothing has to name it.
using byte_master = blocking_master<spi1, std::uint8_t, 1>;
static_assert(emb::spi::some_device<byte_master>);
static_assert(std::same_as<byte_master::frame_type, std::uint8_t>);

using wide_master = blocking_master<spi3, std::uint16_t, 1>;
static_assert(emb::spi::some_device<wide_master>);
static_assert(std::same_as<wide_master::frame_type, std::uint16_t>);

// With several slaves it is not: a driver handed this one would have no way
// to say which part it is talking to, so the concept has to refuse it.
using shared_bus = blocking_master<spi2, std::uint8_t, 2>;
static_assert(!emb::spi::some_device<shared_bus>);

static_assert(detail::calculate_prescaler(emb::units::hz_f32{84000000},
                                          emb::units::hz_f32{20000000})
              == baudrate_prescaler::div8);

static_assert(detail::calculate_prescaler(emb::units::hz_f32{42000000},
                                          emb::units::hz_f32{1000000})
              == baudrate_prescaler::div64);

} // namespace
