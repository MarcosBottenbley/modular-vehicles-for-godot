// Tests for src/math/mv_steering.hpp against the [steering] golden values in
// tests/unit/golden/golden_values.txt.
//
// Terms:
//   golden value: see test_helpers.hpp.
//   arcade car: addons/gevp/scenes/arcade_car.tscn, wheels at x = +-0.5 m and z = +-0.66 m,
//     so wheelbase 1.32 m and half track 0.5 m, maximum steering angle 40 degrees, toe 0.01 rad.
//   left / right wheel: vehicle.gd stores +ackermann and -toe on the left wheel of an axle,
//     -ackermann and +toe on the right one.
//   saturation: the port's guard that caps the Ackermann correction at (pi/2) / theta - 1 once
//     the inner wheel would pass 90 degrees (the raw GDScript formula flips sign there).
//   Ackermann correction / steering input / steering ratio / steering exponent / countersteer
//     correction / dominant slip angle: as defined in mv_steering.hpp.
#include "test_helpers.hpp"

#include "math/mv_steering.hpp"

using namespace mv;

namespace {
// Arcade car maximum steering angle, in rad.
const Real kMaxSteer = deg_to_rad(40);
// Arcade car front Ackermann correction, unitless (golden: ackermann.arcade_L1.32_h0.5_40deg).
constexpr Real kArcadeAckermann = R(0.2722517983);
// Arcade car toe, in rad.
constexpr Real kToe = R(0.01);
} // namespace

TEST_CASE("mv_steering: ackermann_correction matches vehicle.gd initialize() where the GDScript formula is defined") {
	CHECK(ackermann_correction(R(1.32), R(0.5), kMaxSteer) == approx(kArcadeAckermann));
	CHECK(ackermann_correction(R(2.6), R(0.8), deg_to_rad(35)) == approx(0.1928205912));
	CHECK(ackermann_correction(1, R(0.75), deg_to_rad(20)) == approx(0.3296993508));
}

TEST_CASE("mv_steering: ackermann_correction guards") {
	const Real saturated = (kPi / 2) / kMaxSteer - 1;
	SUBCASE("tiny wheelbase, half track or angle return 0") {
		CHECK(ackermann_correction(0, R(0.5), kMaxSteer) == 0);
		CHECK(ackermann_correction(R(1.32), 0, kMaxSteer) == 0);
		CHECK(ackermann_correction(R(1.32), R(0.5), 0) == 0);
		CHECK(ackermann_correction(R(0.000001), R(0.5), kMaxSteer) == 0);
	}
	SUBCASE("saturates once the inner wheel would pass 90 degrees") {
		// d = 1 - 2 * tan(40 deg) < 0: the raw GDScript formula flips sign here.
		CHECK(ackermann_correction(1, 2, kMaxSteer) == approx(saturated));
		// d exactly at the 1e-4 cut-off.
		const Real half_track_at_cutoff = (1 - R(1e-4)) / std::tan(kMaxSteer);
		CHECK(ackermann_correction(1, half_track_at_cutoff, kMaxSteer) == approx(saturated));
	}
	SUBCASE("continuous across the saturation cut-off") {
		const Real half_track_just_before = (1 - R(2e-4)) / std::tan(kMaxSteer);
		CHECK(ackermann_correction(1, half_track_just_before, kMaxSteer) == approx(saturated).epsilon(1e-3));
		CHECK(ackermann_correction(1, half_track_just_before, kMaxSteer) <= saturated);
	}
	SUBCASE("a wider track gives more correction than a narrower one") {
		CHECK(ackermann_correction(R(2.6), R(0.8), kMaxSteer) > ackermann_correction(R(2.6), R(0.4), kMaxSteer));
	}
}

TEST_CASE("mv_steering: steering_angle matches wheel.gd steer()") {
	CHECK(steering_angle(1, 1, kMaxSteer, kArcadeAckermann, -kToe) == approx(0.8781993118));
	CHECK(steering_angle(1, 1, kMaxSteer, -kArcadeAckermann, kToe) == approx(0.5180640898));
	CHECK(steering_angle(R(0.5), 1, kMaxSteer, kArcadeAckermann, -kToe) == approx(0.3947353648));
	CHECK(steering_angle(R(0.5), 1, kMaxSteer, -kArcadeAckermann, kToe) == approx(0.3033963360));
	CHECK(steering_angle(R(-0.25), 1, kMaxSteer, kArcadeAckermann, -kToe) == approx(-0.1700648898));
	CHECK(steering_angle(R(-0.25), 1, kMaxSteer, -kArcadeAckermann, kToe) == approx(-0.1790009606));
	SUBCASE("no input leaves only toe") {
		CHECK(steering_angle(0, 1, kMaxSteer, kArcadeAckermann, -kToe) == approx(-0.01));
		CHECK(steering_angle(0, 1, kMaxSteer, -kArcadeAckermann, kToe) == approx(0.01));
	}
	SUBCASE("a zero steering ratio (rear axle) ignores the input") {
		CHECK(steering_angle(R(0.5), 0, kMaxSteer, kArcadeAckermann, -kToe) == approx(-0.01));
	}
}

TEST_CASE("mv_steering: process_steering() helpers") {
	SUBCASE("steering_exponent_adjust") {
		CHECK(steering_exponent_adjust(R(0.5), R(1.5)) == approx(0.3535533906));
		CHECK(steering_exponent_adjust(R(-0.5), R(1.5)) == approx(-0.3535533906));
		CHECK(steering_exponent_adjust(1, R(1.5)) == approx(1.0));
		CHECK(steering_exponent_adjust(0, R(1.5)) == approx(0.0));
	}
	SUBCASE("steering_speed_correction") {
		CHECK(steering_speed_correction(R(4.25), 20, R(0.2), kMaxSteer) == approx(1.5219191433));
		CHECK(steering_speed_correction(R(4.25), 0, R(0.2), kMaxSteer) == 0);
		CHECK(steering_speed_correction(R(4.25), 20, 0, kMaxSteer) == 0);
	}
	SUBCASE("countersteer_correction") {
		const Real steering_adjust = steering_exponent_adjust(R(0.3), R(1.5));
		CHECK(steering_adjust == approx(0.1643167673));
		CHECK(countersteer_correction(steering_adjust, R(0.1321637183), kMaxSteer, R(0.9)) == approx(0.0996939824));
		// Full lock leaves no headroom for correction.
		CHECK(countersteer_correction(1, R(0.5), kMaxSteer, R(0.9)) == approx(0.0));
		// The asin argument is clamped, so a slightly out-of-range input does not produce NaN.
		CHECK(countersteer_correction(0, R(1.0000001), kMaxSteer, R(0.9)) == approx(kMaxSteer * 0.9));
	}
	SUBCASE("dominant_slip_angle folds like get_max_steering_slip_angle()") {
		Real steering_slip = 0;
		for (Real wheel_slip_angle : { R(0.05), R(-0.12), R(0.1) }) {
			steering_slip = dominant_slip_angle(steering_slip, wheel_slip_angle);
		}
		CHECK(steering_slip == approx(-0.12));
		// Equal magnitudes keep the current value.
		CHECK(dominant_slip_angle(R(0.2), R(-0.2)) == approx(0.2));
	}
}
