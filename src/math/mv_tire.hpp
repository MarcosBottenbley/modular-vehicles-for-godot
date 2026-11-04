// Modular Vehicles for Godot: brush tire model ported 1:1 from addons/gevp/scripts/wheel.gd
// (initialize, process_tires, process_rolling_resistance, get_friction).
//
// The GDScript describes it as "a modified version of the brush tire model that removes the
// friction falloff beyond the peak grip level". Every function below is pure; tire_force()
// chains them in the same order as process_tires() for callers that want the whole model in
// one call. Naming factors is safe, reordering them is not: the golden values depend on the
// GDScript's evaluation order, which is kept everywhere.
//
// Terms:
//   local velocity: the wheel's velocity in its ray frame, in m/s: x right, y up, z back, so a
//     wheel rolling forward has negative z.
//   spin: the wheel's angular velocity, in rad/s; positive drives the wheel forward.
//   wheel surface velocity: spin * tire radius, in m/s, the speed of the tread over the hub.
//   spin velocity difference: wheel surface velocity + local velocity z, in m/s; zero when the
//     tread rolls without sliding, positive when the wheel spins faster than the ground.
//   forward sign: +1 when the wheel moves forward (local z <= 0), -1 when it moves back.
//   slip (slip vector): Vec2(lateral x, longitudinal y), unitless. x is the slip angle in rad
//     (asin of the sideways share of the clamped planar velocity); y is the longitudinal slip
//     ratio of wheel.gd, (|v.z| - surface velocity * forward sign) / (1 + |v.z|): positive when
//     braking, negative when driving.
//   slip guard: wheel.gd replaces an approximately zero slip vector with (0.0001, 0.0001) so
//     the deflection never divides by zero.
//   normal force (spring force): the total suspension force pressing the tire down, in N.
//   tire stiffness: the surface-dependent stiffness of the tread, in N/m^2 (the GDScript's
//     1e6 + 8e6 * surface factor).
//   cornering stiffness: 0.5 * tire stiffness * contact patch^2, in N, the slope of force
//     against slip in the adhesion branch.
//   contact patch: the length of the tread in contact with the ground, in m.
//   tire width: the tread width, in mm as in the GDScript's tire_width.
//   friction (friction limit): the grip the normal force affords, in N: coefficient of
//     friction * normal force minus a pressure penalty normal force / (width * patch * 0.2).
//   deflection: 1 / |cornering stiffness * slip|, in 1/N, how far the brush bends per unit
//     force; the brush model's sliding factor is built from it.
//   critical length: the part of the contact patch still adhering, in m; when it covers the
//     whole patch the tire is in the adhesion (linear) branch, otherwise in the sliding branch.
//   adhesion branch / sliding branch: the two cases of brush_tire_force().
//   braking help: a unitless multiplier > 1 on the longitudinal force while braking with
//     slip.y above 0.3, from the owner's braking grip multiplier.
//   lateral grip assist: a unitless surface factor that raises lateral grip with slip angle.
//   longitudinal grip ratio: a unitless surface factor scaling longitudinal against lateral grip.
//   rolling resistance: the force opposing rolling, in N, from a speed-dependent coefficient,
//     the normal force and a unitless surface factor.
//   force cap: the per-tick upper bounds on each force component: the longitudinal cap from the
//     applied torque or the needed rolling force, the lateral cap from the mass over the wheel.
//   applied torque: |drive torque - brake torque| the wheel received this tick, in N*m.
//   needed rolling force: the force that would cancel the spin velocity difference within one
//     tick, in N.
//   mass over wheel: the share of the vehicle mass resting on this wheel, in kg.
//   limit spin flag: wheel.gd's limit_spin, true when the longitudinal force hit its cap; the
//     vehicle reads it as "this wheel is not slipping".
//   wheel moment: the wheel's moment of inertia about its axle, in kg*m^2.
#ifndef MV_TIRE_HPP
#define MV_TIRE_HPP

#include "mv_math.hpp"

namespace mv {

// wheel.gd initialize(): a solid disc, I = 0.5 * m * r^2 (unitless factor).
constexpr Real kSolidDiscInertiaFactor = static_cast<Real>(0.5);
// wheel.gd initialize(): tire stiffness = 1e6 + 8e6 * surface stiffness, both in N/m^2.
constexpr Real kBaseTireStiffness = static_cast<Real>(1000000);
constexpr Real kTireStiffnessPerSurfaceUnit = static_cast<Real>(8000000);
// wheel.gd process_tires(): the lateral slip saturates at this planar speed, in m/s.
constexpr Real kLateralSlipSaturationSpeed = static_cast<Real>(1);
// wheel.gd process_tires(): the slip guard value for both components, unitless.
constexpr Real kSlipGuard = static_cast<Real>(0.0001);
// wheel.gd process_tires(): cornering stiffness = 0.5 * stiffness * patch^2 (unitless factor).
constexpr Real kCorneringStiffnessFactor = static_cast<Real>(0.5);
// wheel.gd process_tires(): the friction pressure penalty divides by width * patch * 0.2.
constexpr Real kFrictionPressureFactor = static_cast<Real>(0.2);
// wheel.gd process_tires(): braking help engages above this longitudinal slip, unitless.
constexpr Real kBrakingHelpSlipThreshold = static_cast<Real>(0.3);
// wheel.gd process_tires(): critical length = friction * (1 - slip.y) * patch * (0.5 * deflect).
constexpr Real kCriticalLengthFactor = static_cast<Real>(0.5);
// wheel.gd process_tires(): sliding factor = (1 - friction * (1 - slip.y) * (0.25 * deflect)) * deflect.
constexpr Real kSlidingDeflectionFactor = static_cast<Real>(0.25);
// wheel.gd process_rolling_resistance(): coefficient = 0.005 + 0.5 * (0.01 + 0.0095 * (v * 0.036)^2).
constexpr Real kRollingResistanceBase = static_cast<Real>(0.005);
constexpr Real kRollingResistanceSpeedHalf = static_cast<Real>(0.5);
constexpr Real kRollingResistanceSpeedBase = static_cast<Real>(0.01);
constexpr Real kRollingResistanceSpeedGain = static_cast<Real>(0.0095);
// m/s to hundreds of km/h (3.6 / 100), the speed scale of the rolling resistance curve.
constexpr Real kMetresPerSecondToHundredKmh = static_cast<Real>(0.036);

// Moment of inertia of the wheel as a solid disc.
// Ported from wheel.gd initialize(): wheel_moment = 0.5 * wheel_mass * tire_radius^2.
// p_wheel_mass: the wheel's mass, in kg.
// p_tire_radius: the tire radius, in m.
// Returns: the wheel moment in kg*m^2; the wheel integrates its spin with it and the tire
// model uses it for the needed rolling force.
inline Real wheel_moment(Real p_wheel_mass, Real p_tire_radius) {
	return kSolidDiscInertiaFactor * p_wheel_mass * p_tire_radius * p_tire_radius;
}

// Tire stiffness for a surface.
// Ported from wheel.gd initialize(): 1000000.0 + 8000000.0 * tire_stiffnesses[surface].
// p_surface_stiffness: the unitless surface factor from the tire settings (10 for Road).
// Returns: the tire stiffness in N/m^2, the input of cornering_stiffness().
inline Real tire_stiffness(Real p_surface_stiffness) {
	return kBaseTireStiffness + kTireStiffnessPerSurfaceUnit * p_surface_stiffness;
}

// Slip angle from the planar part of the local velocity. The direction is normalised but the
// magnitude is the full 3D speed clamped to 1 m/s, exactly as the GDScript writes it, so the
// angle fades in below 1 m/s instead of jumping at standstill.
// Ported from wheel.gd process_tires(): asin(clamp(-planar.x, -1, 1)).
// p_local_velocity: the local velocity, in m/s.
// Returns: the lateral slip (slip.x) in rad, positive when the wheel slides to its left.
inline Real lateral_slip(const Vec3 &p_local_velocity) {
	const Real clamped_speed = clamp(p_local_velocity.length(), 0, kLateralSlipSaturationSpeed);
	const Vec2 local_planar = Vec2(p_local_velocity.x, p_local_velocity.z).normalized() * clamped_speed;
	return std::asin(clamp(-local_planar.x, -1, 1));
}

// The forward sign of the wheel's motion.
// Ported from wheel.gd process_tires(): z_sign = sign(-local_velocity.z), 1 when exactly 0.
// p_local_velocity_z: the local velocity's z component, in m/s.
// Returns: +1 moving forward or stopped, -1 moving backwards; longitudinal_slip() and the
// sliding branch use it to orient the longitudinal force.
inline Real forward_sign(Real p_local_velocity_z) {
	return p_local_velocity_z == 0 ? static_cast<Real>(1) : sign(-p_local_velocity_z);
}

// Speed of the tread over the hub.
// Ported from wheel.gd process_tires(): wheel_velocity = spin * tire_radius.
// p_spin: the spin, in rad/s.
// p_tire_radius: the tire radius, in m.
// Returns: the wheel surface velocity in m/s.
inline Real wheel_surface_velocity(Real p_spin, Real p_tire_radius) {
	return p_spin * p_tire_radius;
}

// How much faster the tread moves than the ground.
// Ported from wheel.gd process_tires(): spin_velocity_diff = wheel_velocity + local_velocity.z.
// p_wheel_velocity: the wheel surface velocity, in m/s.
// p_local_velocity_z: the local velocity's z component, in m/s (negative going forward).
// Returns: the spin velocity difference in m/s; the wheel keeps it for ABS and the vehicle for
// traction control.
inline Real spin_velocity_difference(Real p_wheel_velocity, Real p_local_velocity_z) {
	return p_wheel_velocity + p_local_velocity_z;
}

// Longitudinal slip ratio.
// Ported from wheel.gd process_tires():
//     slip.y = (abs(v.z) - wheel_velocity * z_sign) / (1 + abs(v.z))
// p_local_velocity_z: the local velocity's z component, in m/s.
// p_wheel_velocity: the wheel surface velocity, in m/s.
// Returns: the longitudinal slip (slip.y), unitless; positive when braking, negative when
// driving, approaching 1 for a locked wheel at speed.
inline Real longitudinal_slip(Real p_local_velocity_z, Real p_wheel_velocity) {
	const Real ground_speed = std::abs(p_local_velocity_z);
	return (ground_speed - (p_wheel_velocity * forward_sign(p_local_velocity_z))) / (static_cast<Real>(1) + ground_speed);
}

// Both slip components with the slip guard applied.
// Ported from wheel.gd process_tires().
// p_local_velocity: the local velocity, in m/s.
// p_spin: the spin, in rad/s.
// p_tire_radius: the tire radius, in m.
// Returns: the slip vector (lateral x in rad, longitudinal y unitless), never approximately
// zero; brush_tire_force() and braking_help() consume it.
inline Vec2 slip_vector(const Vec3 &p_local_velocity, Real p_spin, Real p_tire_radius) {
	const Real wheel_velocity = wheel_surface_velocity(p_spin, p_tire_radius);
	Vec2 slip(lateral_slip(p_local_velocity), longitudinal_slip(p_local_velocity.z, wheel_velocity));
	if (slip.is_zero_approx()) {
		slip = Vec2(kSlipGuard, kSlipGuard);
	}
	return slip;
}

// Cornering stiffness of the tread.
// Ported from wheel.gd process_tires(): 0.5 * tire_stiffness * contact_patch^2.
// p_tire_stiffness: the tire stiffness, in N/m^2.
// p_contact_patch: the contact patch, in m.
// Returns: the cornering stiffness in N, the slope of the adhesion branch.
inline Real cornering_stiffness(Real p_tire_stiffness, Real p_contact_patch) {
	return kCorneringStiffnessFactor * p_tire_stiffness * p_contact_patch * p_contact_patch;
}

// Friction limit of the tire for a normal force.
// Ported from wheel.gd process_tires() / get_friction():
//     cof * normal_force - normal_force / (tire_width * contact_patch * 0.2)
// Guard added for the port: a (near) zero denominator drops the pressure penalty instead of
// dividing by zero.
// p_coefficient_of_friction: the surface's coefficient of friction, unitless.
// p_normal_force: the normal force, in N.
// p_tire_width: the tire width, in mm.
// p_contact_patch: the contact patch, in m.
// Returns: the friction in N; brush_tire_force() scales the sliding branch by it.
inline Real tire_friction(Real p_coefficient_of_friction, Real p_normal_force, Real p_tire_width, Real p_contact_patch) {
	const Real pressure_area = p_tire_width * p_contact_patch * kFrictionPressureFactor;
	if (std::abs(pressure_area) <= kEpsilon) {
		return p_coefficient_of_friction * p_normal_force;
	}
	return p_coefficient_of_friction * p_normal_force - (p_normal_force / pressure_area);
}

// Brush deflection for a slip vector.
// Ported from wheel.gd process_tires():
//     deflect = 1 / sqrt((cs * slip.y)^2 + (cs * slip.x)^2)
// Guard added for the port: returns 0 for an exactly zero slip or stiffness (which also zeroes
// the brush forces downstream) instead of dividing by zero.
// p_cornering_stiffness: the cornering stiffness, in N.
// p_slip: the slip vector.
// Returns: the deflection in 1/N; brush_tire_force() builds the critical length and the
// sliding factor from it.
inline Real brush_deflection(Real p_cornering_stiffness, const Vec2 &p_slip) {
	const Real longitudinal_load = p_cornering_stiffness * p_slip.y;
	const Real lateral_load = p_cornering_stiffness * p_slip.x;
	const Real load_magnitude = std::sqrt(longitudinal_load * longitudinal_load + lateral_load * lateral_load);
	if (load_magnitude <= 0) {
		return 0;
	}
	return static_cast<Real>(1) / load_magnitude;
}

// Braking help multiplier.
// Ported from wheel.gd process_tires(): 1 + braking_grip_multiplier * clamp(abs(slip.y), 0, 1)
// while braking with slip.y > 0.3, else 1.
// p_slip_y: the longitudinal slip, unitless.
// p_braking: true while the driver brakes (vehicle.gd is_braking).
// p_braking_grip_multiplier: the owner's braking grip multiplier, unitless (1.5 by default).
// Returns: the braking help, unitless >= 1, applied to the longitudinal force in the sliding
// branch.
inline Real braking_help(Real p_slip_y, bool p_braking, Real p_braking_grip_multiplier) {
	if (p_slip_y > kBrakingHelpSlipThreshold && p_braking) {
		return static_cast<Real>(1) + (p_braking_grip_multiplier * clamp(std::abs(p_slip_y), 0, 1));
	}
	return 1;
}

// The force that would bring the tread's speed in line with the ground within one tick.
// Ported from wheel.gd process_tires(): ((spin_velocity_diff * wheel_moment) / tire_radius) / delta.
// p_spin_velocity_diff: the spin velocity difference, in m/s.
// p_wheel_moment: the wheel moment, in kg*m^2.
// p_tire_radius: the tire radius, in m. Zero or negative returns 0.
// p_delta: the physics tick, in s. Zero or negative returns 0.
// Returns: the needed rolling force in N (signed like the difference); one candidate for the
// longitudinal force cap.
inline Real needed_rolling_force(Real p_spin_velocity_diff, Real p_wheel_moment, Real p_tire_radius, Real p_delta) {
	if (p_tire_radius <= 0 || p_delta <= 0) {
		return 0;
	}
	return ((p_spin_velocity_diff * p_wheel_moment) / p_tire_radius) / p_delta;
}

// Longitudinal force cap: the larger of the applied torque and the needed rolling force,
// divided by the radius, so the tire never pushes harder than the motor or brakes asked.
// Ported from wheel.gd process_tires() (the max_y_force block).
// p_applied_torque: the applied torque, in N*m.
// p_needed_rolling_force: from needed_rolling_force(), in N.
// p_tire_radius: the tire radius, in m. Zero or negative returns 0.
// Returns: the cap in N, always >= 0; limit_tire_force() applies it to |force.y|.
inline Real max_longitudinal_force(Real p_applied_torque, Real p_needed_rolling_force, Real p_tire_radius) {
	if (p_tire_radius <= 0) {
		return 0;
	}
	if (std::abs(p_applied_torque) > std::abs(p_needed_rolling_force)) {
		return std::abs(p_applied_torque / p_tire_radius);
	}
	return std::abs(p_needed_rolling_force / p_tire_radius);
}

// Lateral force cap: the force that would stop the sideways motion of the mass over the wheel
// within one tick, so a slow sideways drift cannot be overcorrected.
// Ported from wheel.gd process_tires(): abs(mass_over_wheel * local_velocity.x) / delta.
// p_mass_over_wheel: the mass over wheel, in kg.
// p_local_velocity_x: the local velocity's x component, in m/s.
// p_delta: the physics tick, in s. Zero or negative returns 0.
// Returns: the cap in N, always >= 0; limit_tire_force() applies it to |force.x|.
inline Real max_lateral_force(Real p_mass_over_wheel, Real p_local_velocity_x, Real p_delta) {
	if (p_delta <= 0) {
		return 0;
	}
	return std::abs(p_mass_over_wheel * p_local_velocity_x) / p_delta;
}

// The brush model proper: adhesion branch while the critical length covers the contact patch,
// sliding branch otherwise. Before the force caps and rolling resistance.
// Ported from wheel.gd process_tires() (the crit_length / brushx block). Guard added for the
// port: a (near) zero `1 - slip.y` is replaced by kEpsilon in the adhesion branch.
// p_slip: the slip vector.
// p_cornering_stiffness: the cornering stiffness, in N.
// p_friction: the friction, in N.
// p_contact_patch: the contact patch, in m.
// p_longitudinal_grip_ratio: the longitudinal grip ratio, unitless.
// p_lateral_grip_assist: the lateral grip assist, unitless.
// p_braking_help: the braking help, unitless.
// p_forward_sign: the forward sign, +1 or -1.
// Returns: Vec2(lateral x, longitudinal y) in N; tire_force() caps it and adds rolling
// resistance.
inline Vec2 brush_tire_force(const Vec2 &p_slip, Real p_cornering_stiffness, Real p_friction, Real p_contact_patch, Real p_longitudinal_grip_ratio, Real p_lateral_grip_assist, Real p_braking_help, Real p_forward_sign) {
	const Real deflect = brush_deflection(p_cornering_stiffness, p_slip);
	Real one_minus_slip_y = static_cast<Real>(1) - p_slip.y;
	// crit_length = friction * (1 - slip.y) * contact_patch * (0.5 * deflect)
	const Real crit_length = p_friction * one_minus_slip_y * p_contact_patch * (kCriticalLengthFactor * deflect);
	Vec2 force;
	if (crit_length >= p_contact_patch) {
		if (std::abs(one_minus_slip_y) < kEpsilon) {
			one_minus_slip_y = kEpsilon;
		}
		force.y = p_cornering_stiffness * p_slip.y / one_minus_slip_y;
		force.x = p_cornering_stiffness * p_slip.x / one_minus_slip_y;
	} else {
		// brushx = (1 - friction * (1 - slip.y) * (0.25 * deflect)) * deflect
		const Real sliding_factor = (static_cast<Real>(1) - p_friction * one_minus_slip_y * (kSlidingDeflectionFactor * deflect)) * deflect;
		const Real lateral_assist = std::abs(p_slip.x * p_lateral_grip_assist) + static_cast<Real>(1);
		force.y = p_friction * p_longitudinal_grip_ratio * p_cornering_stiffness * p_slip.y * sliding_factor * p_braking_help * p_forward_sign;
		force.x = p_friction * p_cornering_stiffness * p_slip.x * sliding_factor * lateral_assist;
	}
	return force;
}

// Clamps the brush force to the per-tick force caps.
// Ported from wheel.gd process_tires() (the max_y_force / max_x_force block).
// p_force: the brush force, in N.
// p_max_longitudinal_force: from max_longitudinal_force(), in N.
// p_max_lateral_force: from max_lateral_force(), in N.
// r_limit_spin: optional; set to the limit spin flag (true when |force.y| was capped). The
//   vehicle's slip detection and traction control read it.
// Returns: the capped force in N, signs preserved.
inline Vec2 limit_tire_force(const Vec2 &p_force, Real p_max_longitudinal_force, Real p_max_lateral_force, bool *r_limit_spin) {
	Vec2 force = p_force;
	bool longitudinal_capped = false;
	if (std::abs(force.y) > std::abs(p_max_longitudinal_force)) {
		force.y = p_max_longitudinal_force * sign(force.y);
		longitudinal_capped = true;
	}
	if (std::abs(force.x) > p_max_lateral_force) {
		force.x = p_max_lateral_force * sign(force.x);
	}
	if (r_limit_spin != nullptr) {
		*r_limit_spin = longitudinal_capped;
	}
	return force;
}

// Rolling resistance magnitude.
// Ported from wheel.gd process_rolling_resistance():
//     (0.005 + 0.5 * (0.01 + 0.0095 * (v.z * 0.036)^2)) * spring_force * surface_rolling_resistance
// p_local_velocity_z: the local velocity's z component, in m/s.
// p_spring_force: the normal force, in N.
// p_surface_rolling_resistance: the surface's rolling resistance factor, unitless.
// Returns: the rolling resistance in N (magnitude only); tire_force() subtracts it times
// sign(v.z) from force.y.
inline Real rolling_resistance_force(Real p_local_velocity_z, Real p_spring_force, Real p_surface_rolling_resistance) {
	const Real speed_hundred_kmh = p_local_velocity_z * kMetresPerSecondToHundredKmh;
	const Real speed_term = kRollingResistanceSpeedBase + (kRollingResistanceSpeedGain * speed_hundred_kmh * speed_hundred_kmh);
	const Real coefficient = kRollingResistanceBase + (kRollingResistanceSpeedHalf * speed_term);
	return coefficient * p_spring_force * p_surface_rolling_resistance;
}

// The whole tire model for one wheel and one tick: slip, brush force, force caps and rolling
// resistance, in the order of wheel.gd process_tires().
// p_local_velocity: the local velocity, in m/s.
// p_spin: the spin, in rad/s.
// p_tire_radius: the tire radius, in m.
// p_wheel_moment: the wheel moment, in kg*m^2.
// p_applied_torque: the applied torque, in N*m.
// p_mass_over_wheel: the mass over wheel, in kg.
// p_spring_force: the normal force, in N.
// p_tire_stiffness: the tire stiffness, in N/m^2 (tire_stiffness()).
// p_contact_patch: the contact patch, in m.
// p_coefficient_of_friction: the surface's coefficient of friction, unitless.
// p_tire_width: the tire width, in mm.
// p_longitudinal_grip_ratio: the longitudinal grip ratio, unitless.
// p_lateral_grip_assist: the lateral grip assist, unitless.
// p_surface_rolling_resistance: the surface's rolling resistance factor, unitless.
// p_braking: true while the driver brakes.
// p_braking_grip_multiplier: the owner's braking grip multiplier, unitless.
// p_delta: the physics tick, in s.
// r_slip: optional; receives the slip vector (wheel.gd slip_vector) for steering assist.
// r_limit_spin: optional; receives the limit spin flag (wheel.gd limit_spin).
// r_spin_velocity_diff: optional; receives the spin velocity difference in m/s (wheel.gd
//   spin_velocity_diff) for ABS.
// Returns: Vec2(lateral x, longitudinal y) in N including rolling resistance; the wheel applies
// x along its right axis and y along its back axis at the contact point.
inline Vec2 tire_force(const Vec3 &p_local_velocity, Real p_spin, Real p_tire_radius, Real p_wheel_moment, Real p_applied_torque, Real p_mass_over_wheel, Real p_spring_force, Real p_tire_stiffness, Real p_contact_patch, Real p_coefficient_of_friction, Real p_tire_width, Real p_longitudinal_grip_ratio, Real p_lateral_grip_assist, Real p_surface_rolling_resistance, bool p_braking, Real p_braking_grip_multiplier, Real p_delta, Vec2 *r_slip = nullptr, bool *r_limit_spin = nullptr, Real *r_spin_velocity_diff = nullptr) {
	const Real wheel_velocity = wheel_surface_velocity(p_spin, p_tire_radius);
	const Real spin_velocity_diff = spin_velocity_difference(wheel_velocity, p_local_velocity.z);
	const Real needed_force = needed_rolling_force(spin_velocity_diff, p_wheel_moment, p_tire_radius, p_delta);
	const Real longitudinal_cap = max_longitudinal_force(p_applied_torque, needed_force, p_tire_radius);
	const Real lateral_cap = max_lateral_force(p_mass_over_wheel, p_local_velocity.x, p_delta);
	const Real z_sign = forward_sign(p_local_velocity.z);
	const Vec2 slip = slip_vector(p_local_velocity, p_spin, p_tire_radius);

	const Real stiffness = cornering_stiffness(p_tire_stiffness, p_contact_patch);
	const Real friction = tire_friction(p_coefficient_of_friction, p_spring_force, p_tire_width, p_contact_patch);
	const Real help = braking_help(slip.y, p_braking, p_braking_grip_multiplier);

	Vec2 force = brush_tire_force(slip, stiffness, friction, p_contact_patch, p_longitudinal_grip_ratio, p_lateral_grip_assist, help, z_sign);
	force = limit_tire_force(force, longitudinal_cap, lateral_cap, r_limit_spin);
	force.y -= rolling_resistance_force(p_local_velocity.z, p_spring_force, p_surface_rolling_resistance) * sign(p_local_velocity.z);

	if (r_slip != nullptr) {
		*r_slip = slip;
	}
	if (r_spin_velocity_diff != nullptr) {
		*r_spin_velocity_diff = spin_velocity_diff;
	}
	return force;
}

} // namespace mv

#endif // MV_TIRE_HPP
