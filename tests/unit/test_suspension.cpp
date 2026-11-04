#include "test_helpers.hpp"

#include "math/mv_suspension.hpp"

using namespace mv;

// Golden values: tests/unit/golden/golden_values.txt, [suspension].
// Demo car: 1500 kg, 0.5 front weight distribution, so vehicle.gd passes
// 1500 * 0.5 * 4.9 = 3675 N as the per-wheel load.
namespace {
constexpr Real kWheelLoad = 3675;
constexpr Real kSpringLength = R(0.15);
constexpr Real kDelta = R(1.0 / 120.0);
// Per-wheel damper coefficients the demo derives from damping ratio 0.4 (see golden file).
constexpr Real kSlowBump = R(2.2633262159);
constexpr Real kFastBump = R(1.1316631079);
constexpr Real kSlowRebound = R(5.0922293743);
constexpr Real kFastRebound = R(2.5461146871);
constexpr Real kFastDampThreshold = 127;
} // namespace

TEST_CASE("mv_suspension: spring_rate matches vehicle.gd calculate_spring_rate()") {
	// The plan's known golden case: 1500 kg, 0.5 distribution, 0.15 m, 0.5 resting -> 49.0 N/mm
	// for a per-wheel load of 3675 N.
	CHECK(spring_rate(kWheelLoad, kSpringLength, R(0.5)) == approx(49.0));
	CHECK(spring_rate(kWheelLoad, R(0.2), R(0.5)) == approx(36.75));
	CHECK(spring_rate(4000, R(0.12), R(0.6)) == approx(55.5555555556));
	CHECK(spring_rate(kWheelLoad, kSpringLength, R(0.8)) == approx(30.625));
	SUBCASE("zero spring length or resting ratio returns 0 instead of dividing by zero") {
		CHECK(spring_rate(kWheelLoad, 0, R(0.5)) == 0);
		CHECK(spring_rate(kWheelLoad, kSpringLength, 0) == 0);
		CHECK(spring_rate(kWheelLoad, -1, R(0.5)) == 0);
	}
}

TEST_CASE("mv_suspension: damping_coefficient matches vehicle.gd calculate_damping()") {
	CHECK(damping_coefficient(49, kWheelLoad, R(0.4)) == approx(3.3948195828));
	CHECK(damping_coefficient(49, kWheelLoad, R(0.6)) == approx(5.0922293743));
	CHECK(damping_coefficient(R(55.5555555556), 4000, R(0.3)) == approx(2.8284271247));
	SUBCASE("non-positive spring rate or load returns 0 instead of sqrt of a negative") {
		CHECK(damping_coefficient(0, kWheelLoad, R(0.4)) == 0);
		CHECK(damping_coefficient(49, -10, R(0.4)) == 0);
	}
}

TEST_CASE("mv_suspension: compression, spring and antiroll forces match wheel.gd process_suspension()") {
	const Real compression = compression_mm(kSpringLength, R(0.09));
	CHECK(compression == approx(60.0));
	CHECK(spring_force(49, compression) == approx(2940.0));
	CHECK(antiroll_force(49 * R(0.25), compression, 45) == approx(183.75));
	CHECK(antiroll_force(49 * R(0.25), 45, compression) == approx(-183.75));
	CHECK(compression_mm(kSpringLength, kSpringLength) == approx(0.0));
}

TEST_CASE("mv_suspension: damper_force matches the damping_force block of wheel.gd") {
	SUBCASE("bump below, at and above the fast-damp threshold") {
		CHECK(damper_force(50, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false) == approx(113.1663107938));
		CHECK(damper_force(127, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false) == approx(287.4424294163));
		CHECK(damper_force(200, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false) == approx(370.0538362957));
	}
	SUBCASE("rebound below and above the fast-damp threshold") {
		CHECK(damper_force(-50, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false) == approx(-254.6114687126));
		CHECK(damper_force(-300, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false) == approx(-1087.1909714029));
	}
	SUBCASE("zero speed") {
		CHECK(damper_force(0, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false) == approx(0.0));
	}
	SUBCASE("bottomed out cancels bump damping and leaves rebound alone") {
		CHECK(damper_force(200, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, true) == approx(0.0));
		CHECK(damper_force(50, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, true) == approx(0.0));
		CHECK(damper_force(-300, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, true) == approx(-1087.1909714029));
	}
	SUBCASE("the fast branch is continuous at the threshold") {
		const Real below = damper_force(kFastDampThreshold, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false);
		const Real above = damper_force(kFastDampThreshold + R(0.001), kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false);
		CHECK(above == approx(below).epsilon(1e-4));
	}
}

TEST_CASE("mv_suspension: bump_stop_force matches the bottom_out_force term of wheel.gd") {
	const Real mass_over_wheel = 1500 * R(0.5) * R(0.5);
	CHECK(mass_over_wheel == approx(375.0));
	CHECK(bump_stop_force(mass_over_wheel, 800, kDelta, R(9.8), 1, 1) == approx(39675.0));
	CHECK(bump_stop_force(mass_over_wheel, 8000, kDelta, R(9.8), R(0.5), R(1.5)) == approx(340256.25));
	SUBCASE("rebound speed contributes nothing, gravity still does") {
		CHECK(bump_stop_force(mass_over_wheel, -300, kDelta, R(9.8), 1, 1) == approx(3675.0));
	}
	SUBCASE("zero delta returns 0 instead of dividing by zero") {
		CHECK(bump_stop_force(mass_over_wheel, 800, 0, R(9.8), 1, 1) == 0);
	}
}

TEST_CASE("mv_suspension: total force and next_max_spring_length match wheel.gd process_suspension()") {
	const Real compression = compression_mm(kSpringLength, R(0.09));
	const Real total = total_suspension_force(spring_force(49, compression), antiroll_force(49 * R(0.25), compression, 45), damper_force(200, kSlowBump, kFastBump, kSlowRebound, kFastRebound, kFastDampThreshold, false), 0);
	CHECK(total == approx(3493.8038362957));
	CHECK(next_max_spring_length(total, 15, 200, kDelta, R(0.09), kSpringLength) == approx(0.0902743355));
	SUBCASE("the result is clamped to the spring length") {
		CHECK(next_max_spring_length(0, 15, -2000, kDelta, R(0.14), kSpringLength) == approx(0.15));
		CHECK(next_max_spring_length(0, 15, 100000, kDelta, R(0.01), kSpringLength) == approx(0.0));
	}
	SUBCASE("total force never goes negative") {
		CHECK(total_suspension_force(100, 0, -500, 0) == 0);
	}
	SUBCASE("zero wheel mass keeps the current length instead of dividing by zero") {
		CHECK(next_max_spring_length(total, 0, 200, kDelta, R(0.09), kSpringLength) == approx(0.09));
	}
}

TEST_CASE("mv_suspension: axle_spring_force matches vehicle.gd calculate_axle_spring_force()") {
	CHECK(axle_spring_force(R(0.6), kSpringLength, 49) == approx(8820.0));
	CHECK(axle_spring_force(R(0.4), R(0.2), R(36.75)) == approx(5880.0));
	// vehicle.gd initialize(): default front brake bias from those two numbers.
	CHECK(8820 / (8820 + R(5880.0)) == approx(0.6));
}
