#include "test_helpers.hpp"

#include "math/mv_tire.hpp"

using namespace mv;

// Golden values: tests/unit/golden/golden_values.txt, [tire.*]. All cases use the vehicle.gd
// defaults (Tires group), the 0.3 m / 15 kg / 245 mm demo wheel and a 120 Hz tick.
namespace {
constexpr Real kTireRadius = R(0.3);
constexpr Real kWheelMass = 15;
constexpr Real kTireWidth = 245;
constexpr Real kContactPatch = R(0.2);
constexpr Real kBrakingGripMultiplier = R(1.5);
constexpr Real kMassOverWheel = 1500 * R(0.5) * R(0.5);
constexpr Real kDelta = R(1.0 / 120.0);

struct Surface {
	Real stiffness;
	Real cof;
	Real rolling_resistance;
	Real lateral_grip_assist;
	Real longitudinal_grip_ratio;
};
constexpr Surface kRoad = { 10, 3, 1, R(0.05), R(0.5) };
constexpr Surface kDirt = { R(0.5), R(2.4), 2, 0, R(0.5) };
constexpr Surface kGrass = { R(0.5), 2, 4, 0, R(0.5) };

struct TireResult {
	Vec2 force;
	Vec2 slip;
	bool limit_spin = false;
	Real spin_velocity_diff = 0;
};

TireResult run_tire(const Vec3 &p_local_velocity, Real p_spin, Real p_applied_torque, Real p_spring_force, bool p_braking, const Surface &p_surface) {
	TireResult r;
	r.force = tire_force(p_local_velocity, p_spin, kTireRadius, wheel_moment(kWheelMass, kTireRadius), p_applied_torque, kMassOverWheel, p_spring_force, tire_stiffness(p_surface.stiffness), kContactPatch, p_surface.cof, kTireWidth, p_surface.longitudinal_grip_ratio, p_surface.lateral_grip_assist, p_surface.rolling_resistance, p_braking, kBrakingGripMultiplier, kDelta, &r.slip, &r.limit_spin, &r.spin_velocity_diff);
	return r;
}
} // namespace

TEST_CASE("mv_tire: wheel_moment and tire_stiffness match wheel.gd initialize()") {
	CHECK(wheel_moment(kWheelMass, kTireRadius) == approx(0.675));
	CHECK(tire_stiffness(10) == approx(81000000.0));
	CHECK(tire_stiffness(R(0.5)) == approx(5000000.0));
	CHECK(tire_stiffness(0) == approx(1000000.0));
}

TEST_CASE("mv_tire: slip computation matches wheel.gd process_tires() [tire.rolling_driven]") {
	const Vec3 v(R(0.1), 0, -10);
	CHECK(lateral_slip(v) == approx(-0.0099996673));
	CHECK(forward_sign(v.z) == 1);
	CHECK(forward_sign(4) == -1);
	CHECK(forward_sign(0) == 1);
	const Real wheel_velocity = wheel_surface_velocity(36, kTireRadius);
	CHECK(wheel_velocity == approx(10.8));
	CHECK(spin_velocity_difference(wheel_velocity, v.z) == approx(0.8));
	CHECK(longitudinal_slip(v.z, wheel_velocity) == approx(-0.0727272704));
	const Vec2 slip = slip_vector(v, 36, kTireRadius);
	CHECK(slip.x == approx(-0.0099996673));
	CHECK(slip.y == approx(-0.0727272704));
	SUBCASE("a locked wheel at speed: slip.y approaches 1") {
		CHECK(longitudinal_slip(-20, 0) == approx(0.9523809552));
	}
	SUBCASE("reversing flips the forward sign [tire.reversing]") {
		CHECK(longitudinal_slip(4, wheel_surface_velocity(-12, kTireRadius)) == approx(0.0799999982));
		CHECK(lateral_slip(Vec3(R(0.5), 0, 4)) == approx(-0.1243549958));
	}
	SUBCASE("a zero slip vector is replaced by (0.0001, 0.0001) [tire.standstill]") {
		const Vec2 s = slip_vector(Vec3(0, 0, 0), 0, kTireRadius);
		CHECK(s.x == approx(0.0001, 1e-9));
		CHECK(s.y == approx(0.0001, 1e-9));
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
	const Real needed = needed_rolling_force(R(0.8), wheel_moment(kWheelMass, kTireRadius), kTireRadius, kDelta);
	CHECK(needed == approx(216.0));
	CHECK(max_longitudinal_force(400, needed, kTireRadius) == approx(1333.3333333333));
	CHECK(max_longitudinal_force(50, needed, kTireRadius) == approx(720.0));
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
		CHECK(max_longitudinal_force(400, needed, 0) == 0);
		CHECK(max_lateral_force(kMassOverWheel, R(0.1), 0) == 0);
	}
}

TEST_CASE("mv_tire: brush_tire_force, limit_tire_force and rolling resistance [tire.rolling_driven]") {
	const Real stiffness = cornering_stiffness(tire_stiffness(kRoad.stiffness), kContactPatch);
	const Real friction = tire_friction(kRoad.cof, 3675, kTireWidth, kContactPatch);
	const Vec2 slip(R(-0.0099996673), R(-0.0727272704));
	const Vec2 brush = brush_tire_force(slip, stiffness, friction, kContactPatch, kRoad.longitudinal_grip_ratio, kRoad.lateral_grip_assist, 1, 1);
	CHECK(brush.x == approx(-1416.5465087891));
	CHECK(brush.y == approx(-5148.6752929688));
	bool limit_spin = false;
	const Vec2 limited = limit_tire_force(brush, R(1333.3333333333), R(4500.0), &limit_spin);
	CHECK(limited.x == approx(-1416.5465087891));
	CHECK(limited.y == approx(-1333.3333333333));
	CHECK(limit_spin);
	CHECK(rolling_resistance_force(-10, 3675, kRoad.rolling_resistance) == approx(39.01233));
	CHECK(limited.y - rolling_resistance_force(-10, 3675, kRoad.rolling_resistance) * sign(R(-10)) == approx(-1294.3210449219));
	SUBCASE("the lateral cap and the limit flag are independent") {
		bool flag = true;
		const Vec2 l = limit_tire_force(Vec2(-5000, -100), 1000, 2000, &flag);
		CHECK(l.x == approx(-2000.0));
		CHECK(l.y == approx(-100.0));
		CHECK_FALSE(flag);
		limit_tire_force(Vec2(-5000, -100), 1000, 2000, nullptr); // out-param is optional
	}
	SUBCASE("linear (adhesion) branch when the critical length covers the patch [tire.standstill]") {
		const Vec2 tiny(R(0.0001), R(0.0001));
		const Vec2 f = brush_tire_force(tiny, stiffness, friction, kContactPatch, kRoad.longitudinal_grip_ratio, kRoad.lateral_grip_assist, 1, 1);
		CHECK(f.x == approx(162.0162048340));
		CHECK(f.y == approx(162.0162048340));
	}
	SUBCASE("1 - slip.y guard in the linear branch does not divide by zero") {
		// contact_patch 0 forces the linear branch; slip.y == 1 would otherwise be x / 0.
		const Vec2 f = brush_tire_force(Vec2(R(0.1), 1), stiffness, friction, 0, kRoad.longitudinal_grip_ratio, kRoad.lateral_grip_assist, 1, 1);
		CHECK(std::isfinite(f.x));
		CHECK(std::isfinite(f.y));
		CHECK(f.y == approx(stiffness / kEpsilon));
	}
	SUBCASE("rolling resistance grows with speed and surface") {
		CHECK(rolling_resistance_force(-20, 3675, kRoad.rolling_resistance) == approx(45.79932));
		CHECK(rolling_resistance_force(R(-0.5), 3675, kGrass.rolling_resistance) == approx(147.0226233));
		CHECK(rolling_resistance_force(0, 0, 1) == approx(0.0));
	}
}

TEST_CASE("mv_tire: tire_force end to end matches wheel.gd process_tires() golden cases") {
	SUBCASE("rolling_driven: slight drive slip, longitudinal force capped by the applied torque") {
		const TireResult r = run_tire(Vec3(R(0.1), 0, -10), 36, 400, 3675, false, kRoad);
		CHECK(r.slip.x == approx(-0.0099996673));
		CHECK(r.slip.y == approx(-0.0727272704));
		CHECK(r.spin_velocity_diff == approx(0.8));
		CHECK(r.force.x == approx(-1416.5465087891));
		CHECK(r.force.y == approx(-1294.3210449219));
		CHECK(r.limit_spin);
	}
	SUBCASE("cornering: pure lateral slip, longitudinal force is rolling resistance only") {
		const TireResult r = run_tire(Vec3(3, 0, -15), 50, 200, 4200, false, kRoad);
		CHECK(r.slip.x == approx(-0.1973955631));
		CHECK(r.slip.y == approx(0.0));
		CHECK(r.spin_velocity_diff == approx(0.0));
		CHECK(r.force.x == approx(-12174.5986328125));
		CHECK(r.force.y == approx(47.8174209595));
		CHECK_FALSE(r.limit_spin);
	}
	SUBCASE("braking_locked: locked wheel at 20 m/s with braking help") {
		const TireResult r = run_tire(Vec3(R(0.2), 0, -20), 0, 3000, 3675, true, kRoad);
		CHECK(r.slip.x == approx(-0.0099996673));
		CHECK(r.slip.y == approx(0.9523809552));
		CHECK(r.spin_velocity_diff == approx(-20.0));
		CHECK(r.force.x == approx(-111.8618316650));
		CHECK(r.force.y == approx(12976.1669921875));
		CHECK_FALSE(r.limit_spin);
	}
	SUBCASE("standstill: slip guard, linear branch, every force capped to zero") {
		const TireResult r = run_tire(Vec3(0, 0, 0), 0, 0, 3675, false, kRoad);
		CHECK(r.slip.x == approx(0.0001, 1e-9));
		CHECK(r.slip.y == approx(0.0001, 1e-9));
		CHECK(r.force.x == approx(0.0));
		CHECK(r.force.y == approx(0.0));
		CHECK(r.limit_spin);
	}
	SUBCASE("reversing: negative forward sign") {
		const TireResult r = run_tire(Vec3(R(0.5), 0, 4), -12, 150, 3000, false, kRoad);
		CHECK(r.slip.x == approx(-0.1243549958));
		CHECK(r.slip.y == approx(0.0799999982));
		CHECK(r.spin_velocity_diff == approx(0.4));
		CHECK(r.force.x == approx(-7295.6127929688));
		CHECK(r.force.y == approx(-530.2954711914));
		CHECK(r.limit_spin);
	}
	SUBCASE("dirt_slide: soft surface, no grip assist, nothing capped") {
		const TireResult r = run_tire(Vec3(2, 0, -8), 40, 600, 3675, false, kDirt);
		CHECK(r.slip.x == approx(-0.2449786812));
		CHECK(r.slip.y == approx(-0.4444444478));
		CHECK(r.force.x == approx(-3831.6564941406));
		CHECK(r.force.y == approx(-3399.3320312500));
		CHECK_FALSE(r.limit_spin);
	}
	SUBCASE("grass_crawl: low speed, both forces capped, heavy rolling resistance") {
		const TireResult r = run_tire(Vec3(R(0.05), 0, R(-0.5)), 2, 50, 3675, false, kGrass);
		CHECK(r.slip.x == approx(-0.0500208586));
		CHECK(r.slip.y == approx(-0.0666666701));
		CHECK(r.force.x == approx(-2250.0));
		CHECK(r.force.y == approx(-19.6440486908));
		CHECK(r.limit_spin);
	}
	SUBCASE("no vertical load gives no tire force beyond the slip guard") {
		const TireResult r = run_tire(Vec3(1, 0, -10), 30, 100, 0, false, kRoad);
		CHECK(std::isfinite(r.force.x));
		CHECK(std::isfinite(r.force.y));
		CHECK(r.force.x == approx(0.0));
		CHECK(r.force.y == approx(0.0));
	}
}
