// Tests for src/math/mv_tire.hpp against the [tire.*] golden values in
// tests/unit/golden/golden_values.txt. Every case uses the demo wheel on one of the three demo
// surfaces; the bracketed name in a test title is the golden section it reads.
//
// Terms:
//   golden value / golden case: see test_helpers.hpp; a golden case is one [tire.<name>]
//     section, a full run of wheel.gd process_tires() for one set of inputs.
//   demo wheel: vehicle.gd defaults, 0.3 m tire radius, 15 kg wheel mass, 245 mm tire width,
//     0.2 m contact patch, braking grip multiplier 1.5, 375 kg mass over wheel (1500 kg,
//     0.5 distribution, half an axle), 120 Hz physics tick.
//   demo surface: one of vehicle.gd's Road / Dirt / Grass dictionaries (stiffness, coefficient
//     of friction, rolling resistance, lateral grip assist, longitudinal grip ratio).
//   local velocity / spin / slip / forward sign / normal force / cornering stiffness /
//     friction / deflection / braking help / force cap / limit spin flag / rolling resistance:
//     as defined in mv_tire.hpp.
#include "test_helpers.hpp"

#include "math/mv_tire.hpp"

using namespace mv;

namespace {
// Demo wheel: tire radius in m, wheel mass in kg, tire width in mm, contact patch in m.
constexpr Real kTireRadius = R(0.3);
constexpr Real kWheelMass = 15;
constexpr Real kTireWidth = 245;
constexpr Real kContactPatch = R(0.2);
// Demo braking grip multiplier, unitless.
constexpr Real kBrakingGripMultiplier = R(1.5);
// Demo mass over wheel, in kg: vehicle_mass * weight_distribution * 0.5.
constexpr Real kMassOverWheel = 1500 * R(0.5) * R(0.5);
// Physics tick at 120 Hz, in s.
constexpr Real kDelta = R(1.0 / 120.0);

// A demo surface: the per-surface entries of vehicle.gd's tire dictionaries.
struct Surface {
	Real stiffness; // tire_stiffnesses[surface], unitless
	Real cof; // coefficient_of_friction[surface], unitless
	Real rolling_resistance; // rolling_resistance[surface], unitless
	Real lateral_grip_assist; // lateral_grip_assist[surface], unitless
	Real longitudinal_grip_ratio; // longitudinal_grip_ratio[surface], unitless
};
constexpr Surface kRoad = { 10, 3, 1, R(0.05), R(0.5) };
constexpr Surface kDirt = { R(0.5), R(2.4), 2, 0, R(0.5) };
constexpr Surface kGrass = { R(0.5), 2, 4, 0, R(0.5) };

// Everything tire_force() reports for one golden case, mirroring wheel.gd's members.
struct TireResult {
	Vec2 force; // force_vector, in N
	Vec2 slip; // slip_vector
	bool limit_spin = false; // limit_spin
	Real spin_velocity_diff = 0; // spin_velocity_diff, in m/s
};

// Runs tire_force() for the demo wheel with the inputs a golden case varies.
// p_local_velocity: the local velocity, in m/s.
// p_spin: the spin, in rad/s.
// p_applied_torque: the applied torque, in N*m.
// p_spring_force: the normal force, in N.
// p_braking: true while braking.
// p_surface: the demo surface.
// Returns: the TireResult to compare with the golden section.
TireResult run_tire(const Vec3 &p_local_velocity, Real p_spin, Real p_applied_torque, Real p_spring_force, bool p_braking, const Surface &p_surface) {
	TireResult result;
	result.force = tire_force(p_local_velocity, p_spin, kTireRadius, wheel_moment(kWheelMass, kTireRadius), p_applied_torque, kMassOverWheel, p_spring_force, tire_stiffness(p_surface.stiffness), kContactPatch, p_surface.cof, kTireWidth, p_surface.longitudinal_grip_ratio, p_surface.lateral_grip_assist, p_surface.rolling_resistance, p_braking, kBrakingGripMultiplier, kDelta, &result.slip, &result.limit_spin, &result.spin_velocity_diff);
	return result;
}
} // namespace

TEST_CASE("mv_tire: wheel_moment and tire_stiffness match wheel.gd initialize()") {
	CHECK(wheel_moment(kWheelMass, kTireRadius) == approx(0.675));
	CHECK(tire_stiffness(10) == approx(81000000.0));
	CHECK(tire_stiffness(R(0.5)) == approx(5000000.0));
	CHECK(tire_stiffness(0) == approx(1000000.0));
}

TEST_CASE("mv_tire: slip computation matches wheel.gd process_tires() [tire.rolling_driven]") {
	const Vec3 local_velocity(R(0.1), 0, -10);
	CHECK(lateral_slip(local_velocity) == approx(-0.0099996673));
	CHECK(forward_sign(local_velocity.z) == 1);
	CHECK(forward_sign(4) == -1);
	CHECK(forward_sign(0) == 1);
	const Real wheel_velocity = wheel_surface_velocity(36, kTireRadius);
	CHECK(wheel_velocity == approx(10.8));
	CHECK(spin_velocity_difference(wheel_velocity, local_velocity.z) == approx(0.8));
	CHECK(longitudinal_slip(local_velocity.z, wheel_velocity) == approx(-0.0727272704));
	const Vec2 slip = slip_vector(local_velocity, 36, kTireRadius);
	CHECK(slip.x == approx(-0.0099996673));
	CHECK(slip.y == approx(-0.0727272704));
	SUBCASE("a locked wheel at speed: slip.y approaches 1") {
		CHECK(longitudinal_slip(-20, 0) == approx(0.9523809552));
	}
	SUBCASE("reversing flips the forward sign [tire.reversing]") {
		CHECK(longitudinal_slip(4, wheel_surface_velocity(-12, kTireRadius)) == approx(0.0799999982));
		CHECK(lateral_slip(Vec3(R(0.5), 0, 4)) == approx(-0.1243549958));
	}
	SUBCASE("a zero slip vector is replaced by the slip guard [tire.standstill]") {
		const Vec2 guarded_slip = slip_vector(Vec3(0, 0, 0), 0, kTireRadius);
		CHECK(guarded_slip.x == approx(0.0001, 1e-9));
		CHECK(guarded_slip.y == approx(0.0001, 1e-9));
	}
	SUBCASE("lateral slip saturates at 1 m/s of planar speed") {
		// asin(-1) when the whole (clamped) velocity is sideways.
		CHECK(lateral_slip(Vec3(5, 0, 0)) == approx(-kPi / 2));
		CHECK(lateral_slip(Vec3(R(-0.5), 0, 0)) == approx(kPi / 6));
	}
}

TEST_CASE("mv_tire: stiffness, friction, deflection and caps match wheel.gd process_tires() [tire.rolling_driven]") {
	const Real stiffness = cornering_stiffness(tire_stiffness(kRoad.stiffness), kContactPatch);
	CHECK(stiffness == approx(1620000.0));
	const Real friction = tire_friction(kRoad.cof, 3675, kTireWidth, kContactPatch);
	CHECK(friction == approx(10650.0));
	const Vec2 slip(R(-0.0099996673), R(-0.0727272704));
	CHECK(brush_deflection(stiffness, slip) == approx(0.0000084085, 1e-3));
	CHECK(braking_help(slip.y, false, kBrakingGripMultiplier) == approx(1.0));
	const Real needed_force = needed_rolling_force(R(0.8), wheel_moment(kWheelMass, kTireRadius), kTireRadius, kDelta);
	CHECK(needed_force == approx(216.0));
	CHECK(max_longitudinal_force(400, needed_force, kTireRadius) == approx(1333.3333333333));
	CHECK(max_longitudinal_force(50, needed_force, kTireRadius) == approx(720.0));
	CHECK(max_lateral_force(kMassOverWheel, R(0.1), kDelta) == approx(4500.0));
	SUBCASE("braking help engages above 0.3 slip only while braking [tire.braking_locked]") {
		CHECK(braking_help(R(0.9523809552), true, kBrakingGripMultiplier) == approx(2.4285714328));
		CHECK(braking_help(R(0.9523809552), false, kBrakingGripMultiplier) == approx(1.0));
		CHECK(braking_help(R(0.2), true, kBrakingGripMultiplier) == approx(1.0));
		CHECK(braking_help(R(1.5), true, kBrakingGripMultiplier) == approx(2.5));
	}
	SUBCASE("guards") {
		CHECK(tire_friction(kRoad.cof, 3675, 0, kContactPatch) == approx(3.0 * 3675.0));
		CHECK(tire_friction(kRoad.cof, 3675, kTireWidth, 0) == approx(3.0 * 3675.0));
		CHECK(brush_deflection(stiffness, Vec2(0, 0)) == 0);
		CHECK(brush_deflection(0, slip) == 0);
		CHECK(needed_rolling_force(R(0.8), R(0.675), 0, kDelta) == 0);
		CHECK(needed_rolling_force(R(0.8), R(0.675), kTireRadius, 0) == 0);
		CHECK(max_longitudinal_force(400, needed_force, 0) == 0);
		CHECK(max_lateral_force(kMassOverWheel, R(0.1), 0) == 0);
	}
}

TEST_CASE("mv_tire: brush_tire_force, limit_tire_force and rolling resistance [tire.rolling_driven]") {
	const Real stiffness = cornering_stiffness(tire_stiffness(kRoad.stiffness), kContactPatch);
	const Real friction = tire_friction(kRoad.cof, 3675, kTireWidth, kContactPatch);
	const Vec2 slip(R(-0.0099996673), R(-0.0727272704));
	const Vec2 brush_force = brush_tire_force(slip, stiffness, friction, kContactPatch, kRoad.longitudinal_grip_ratio, kRoad.lateral_grip_assist, 1, 1);
	CHECK(brush_force.x == approx(-1416.5465087891));
	CHECK(brush_force.y == approx(-5148.6752929688));
	bool limit_spin = false;
	const Vec2 limited_force = limit_tire_force(brush_force, R(1333.3333333333), R(4500.0), &limit_spin);
	CHECK(limited_force.x == approx(-1416.5465087891));
	CHECK(limited_force.y == approx(-1333.3333333333));
	CHECK(limit_spin);
	const Real rolling_resistance = rolling_resistance_force(-10, 3675, kRoad.rolling_resistance);
	CHECK(rolling_resistance == approx(39.01233));
	CHECK(limited_force.y - rolling_resistance * sign(R(-10)) == approx(-1294.3210449219));
	SUBCASE("the lateral cap and the limit spin flag are independent") {
		bool flag = true;
		const Vec2 capped = limit_tire_force(Vec2(-5000, -100), 1000, 2000, &flag);
		CHECK(capped.x == approx(-2000.0));
		CHECK(capped.y == approx(-100.0));
		CHECK_FALSE(flag);
		limit_tire_force(Vec2(-5000, -100), 1000, 2000, nullptr); // the out-param is optional
	}
	SUBCASE("adhesion branch when the critical length covers the patch [tire.standstill]") {
		const Vec2 guarded_slip(R(0.0001), R(0.0001));
		const Vec2 adhesion_force = brush_tire_force(guarded_slip, stiffness, friction, kContactPatch, kRoad.longitudinal_grip_ratio, kRoad.lateral_grip_assist, 1, 1);
		CHECK(adhesion_force.x == approx(162.0162048340));
		CHECK(adhesion_force.y == approx(162.0162048340));
	}
	SUBCASE("the 1 - slip.y guard in the adhesion branch does not divide by zero") {
		// contact_patch 0 forces the adhesion branch; slip.y == 1 would otherwise be x / 0.
		const Vec2 locked_slip(R(0.1), 1);
		const Vec2 guarded_force = brush_tire_force(locked_slip, stiffness, friction, 0, kRoad.longitudinal_grip_ratio, kRoad.lateral_grip_assist, 1, 1);
		CHECK(std::isfinite(guarded_force.x));
		CHECK(std::isfinite(guarded_force.y));
		CHECK(guarded_force.y == approx(stiffness / kEpsilon));
	}
	SUBCASE("rolling resistance grows with speed and surface") {
		CHECK(rolling_resistance_force(-20, 3675, kRoad.rolling_resistance) == approx(45.79932));
		CHECK(rolling_resistance_force(R(-0.5), 3675, kGrass.rolling_resistance) == approx(147.0226233));
		CHECK(rolling_resistance_force(0, 0, 1) == approx(0.0));
	}
}

TEST_CASE("mv_tire: tire_force end to end matches wheel.gd process_tires() golden cases") {
	SUBCASE("rolling_driven: slight drive slip, longitudinal force capped by the applied torque") {
		const TireResult result = run_tire(Vec3(R(0.1), 0, -10), 36, 400, 3675, false, kRoad);
		CHECK(result.slip.x == approx(-0.0099996673));
		CHECK(result.slip.y == approx(-0.0727272704));
		CHECK(result.spin_velocity_diff == approx(0.8));
		CHECK(result.force.x == approx(-1416.5465087891));
		CHECK(result.force.y == approx(-1294.3210449219));
		CHECK(result.limit_spin);
	}
	SUBCASE("cornering: pure lateral slip, longitudinal force is rolling resistance only") {
		const TireResult result = run_tire(Vec3(3, 0, -15), 50, 200, 4200, false, kRoad);
		CHECK(result.slip.x == approx(-0.1973955631));
		CHECK(result.slip.y == approx(0.0));
		CHECK(result.spin_velocity_diff == approx(0.0));
		CHECK(result.force.x == approx(-12174.5986328125));
		CHECK(result.force.y == approx(47.8174209595));
		CHECK_FALSE(result.limit_spin);
	}
	SUBCASE("braking_locked: locked wheel at 20 m/s with braking help") {
		const TireResult result = run_tire(Vec3(R(0.2), 0, -20), 0, 3000, 3675, true, kRoad);
		CHECK(result.slip.x == approx(-0.0099996673));
		CHECK(result.slip.y == approx(0.9523809552));
		CHECK(result.spin_velocity_diff == approx(-20.0));
		CHECK(result.force.x == approx(-111.8618316650));
		CHECK(result.force.y == approx(12976.1669921875));
		CHECK_FALSE(result.limit_spin);
	}
	SUBCASE("standstill: slip guard, adhesion branch, every force capped to zero") {
		const TireResult result = run_tire(Vec3(0, 0, 0), 0, 0, 3675, false, kRoad);
		CHECK(result.slip.x == approx(0.0001, 1e-9));
		CHECK(result.slip.y == approx(0.0001, 1e-9));
		CHECK(result.force.x == approx(0.0));
		CHECK(result.force.y == approx(0.0));
		CHECK(result.limit_spin);
	}
	SUBCASE("reversing: negative forward sign") {
		const TireResult result = run_tire(Vec3(R(0.5), 0, 4), -12, 150, 3000, false, kRoad);
		CHECK(result.slip.x == approx(-0.1243549958));
		CHECK(result.slip.y == approx(0.0799999982));
		CHECK(result.spin_velocity_diff == approx(0.4));
		CHECK(result.force.x == approx(-7295.6127929688));
		CHECK(result.force.y == approx(-530.2954711914));
		CHECK(result.limit_spin);
	}
	SUBCASE("dirt_slide: soft surface, no grip assist, nothing capped") {
		const TireResult result = run_tire(Vec3(2, 0, -8), 40, 600, 3675, false, kDirt);
		CHECK(result.slip.x == approx(-0.2449786812));
		CHECK(result.slip.y == approx(-0.4444444478));
		CHECK(result.force.x == approx(-3831.6564941406));
		CHECK(result.force.y == approx(-3399.3320312500));
		CHECK_FALSE(result.limit_spin);
	}
	SUBCASE("grass_crawl: low speed, both forces capped, heavy rolling resistance") {
		const TireResult result = run_tire(Vec3(R(0.05), 0, R(-0.5)), 2, 50, 3675, false, kGrass);
		CHECK(result.slip.x == approx(-0.0500208586));
		CHECK(result.slip.y == approx(-0.0666666701));
		CHECK(result.force.x == approx(-2250.0));
		CHECK(result.force.y == approx(-19.6440486908));
		CHECK(result.limit_spin);
	}
	SUBCASE("no vertical load gives no tire force beyond the slip guard") {
		const TireResult result = run_tire(Vec3(1, 0, -10), 30, 100, 0, false, kRoad);
		CHECK(std::isfinite(result.force.x));
		CHECK(std::isfinite(result.force.y));
		CHECK(result.force.x == approx(0.0));
		CHECK(result.force.y == approx(0.0));
	}
}
