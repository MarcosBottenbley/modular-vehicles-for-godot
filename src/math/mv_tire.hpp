// Modular Vehicles for Godot: brush tire model ported 1:1 from addons/gevp/scripts/wheel.gd
// (initialize, process_tires, process_rolling_resistance, get_friction).
//
// The GDScript describes it as "a modified version of the brush tire model that removes the
// friction falloff beyond the peak grip level". Conventions: the wheel's local velocity is in
// the ray's frame (x right, y up, z back; forward is -Z), spin is rad/s with positive spin
// driving the wheel forward, slip and force vectors are Vec2(lateral x, longitudinal y),
// forces in N. Every function below is pure; tire_force() chains them in the same order as
// process_tires() for callers that want the whole model in one call.
#ifndef MV_TIRE_HPP
#define MV_TIRE_HPP

#include "mv_math.hpp"

namespace mv {

// Ported from wheel.gd initialize(): `wheel_moment = 0.5 * wheel_mass * tire_radius^2`.
inline Real wheel_moment(Real p_wheel_mass, Real p_tire_radius) {
	return static_cast<Real>(0.5) * p_wheel_mass * p_tire_radius * p_tire_radius;
}

// Ported from wheel.gd initialize(): `1000000.0 + 8000000.0 * tire_stiffnesses[surface]`.
inline Real tire_stiffness(Real p_surface_stiffness) {
	return static_cast<Real>(1000000) + static_cast<Real>(8000000) * p_surface_stiffness;
}

// Ported from wheel.gd process_tires(): the planar velocity direction scaled by the clamped
// speed, then `asin(clamp(-planar.x, -1, 1))`. Note the GDScript normalises the 2D planar
// vector but clamps the full 3D speed; this keeps that.
inline Real lateral_slip(const Vec3 &p_local_velocity) {
	const Vec2 local_planar = Vec2(p_local_velocity.x, p_local_velocity.z).normalized() * clamp(p_local_velocity.length(), 0, 1);
	return std::asin(clamp(-local_planar.x, -1, 1));
}

// Ported from wheel.gd process_tires(): `z_sign = sign(-local_velocity.z)`, 1 when exactly 0.
inline Real forward_sign(Real p_local_velocity_z) {
	return p_local_velocity_z == 0 ? static_cast<Real>(1) : sign(-p_local_velocity_z);
}

// Ported from wheel.gd process_tires(): `wheel_velocity = spin * tire_radius` (m/s).
inline Real wheel_surface_velocity(Real p_spin, Real p_tire_radius) {
	return p_spin * p_tire_radius;
}

// Ported from wheel.gd process_tires(): `spin_velocity_diff = wheel_velocity + local_velocity.z`.
inline Real spin_velocity_difference(Real p_wheel_velocity, Real p_local_velocity_z) {
	return p_wheel_velocity + p_local_velocity_z;
}

// Ported from wheel.gd process_tires():
//     slip.y = (abs(v.z) - wheel_velocity * z_sign) / (1 + abs(v.z))
inline Real longitudinal_slip(Real p_local_velocity_z, Real p_wheel_velocity) {
	const Real abs_z = std::abs(p_local_velocity_z);
	return (abs_z - (p_wheel_velocity * forward_sign(p_local_velocity_z))) / (static_cast<Real>(1) + abs_z);
}

// Ported from wheel.gd process_tires(): both slip components, with the GDScript's guard that
// replaces an (approximately) zero slip vector by (0.0001, 0.0001).
inline Vec2 slip_vector(const Vec3 &p_local_velocity, Real p_spin, Real p_tire_radius) {
	Vec2 slip(lateral_slip(p_local_velocity), longitudinal_slip(p_local_velocity.z, wheel_surface_velocity(p_spin, p_tire_radius)));
	if (slip.is_zero_approx()) {
		slip = Vec2(static_cast<Real>(0.0001), static_cast<Real>(0.0001));
	}
	return slip;
}

// Ported from wheel.gd process_tires(): `0.5 * tire_stiffness * contact_patch^2`.
inline Real cornering_stiffness(Real p_tire_stiffness, Real p_contact_patch) {
	return static_cast<Real>(0.5) * p_tire_stiffness * p_contact_patch * p_contact_patch;
}

// Ported from wheel.gd process_tires() / get_friction():
//     cof * normal_force - normal_force / (tire_width * contact_patch * 0.2)
// tire_width is in millimetres as in the GDScript. Guard added for the port: when the
// denominator is (near) zero the correction term is dropped instead of dividing by zero.
inline Real tire_friction(Real p_coefficient_of_friction, Real p_normal_force, Real p_tire_width, Real p_contact_patch) {
	const Real denominator = p_tire_width * p_contact_patch * static_cast<Real>(0.2);
	if (std::abs(denominator) <= kEpsilon) {
		return p_coefficient_of_friction * p_normal_force;
	}
	return p_coefficient_of_friction * p_normal_force - (p_normal_force / denominator);
}

// Ported from wheel.gd process_tires():
//     deflect = 1 / sqrt((cs * slip.y)^2 + (cs * slip.x)^2)
// Guard added for the port: returns 0 when the slip is exactly zero (which also zeroes the
// brush forces downstream) instead of dividing by zero.
inline Real brush_deflection(Real p_cornering_stiffness, const Vec2 &p_slip) {
	const Real sy = p_cornering_stiffness * p_slip.y;
	const Real sx = p_cornering_stiffness * p_slip.x;
	const Real magnitude = std::sqrt(sy * sy + sx * sx);
	if (magnitude <= 0) {
		return 0;
	}
	return static_cast<Real>(1) / magnitude;
}

// Ported from wheel.gd process_tires(): extra longitudinal grip while braking with slip.y > 0.3,
// `1 + braking_grip_multiplier * clamp(abs(slip.y), 0, 1)`; 1 otherwise.
inline Real braking_help(Real p_slip_y, bool p_braking, Real p_braking_grip_multiplier) {
	if (p_slip_y > static_cast<Real>(0.3) && p_braking) {
		return static_cast<Real>(1) + (p_braking_grip_multiplier * clamp(std::abs(p_slip_y), 0, 1));
	}
	return 1;
}

// Ported from wheel.gd process_tires(): the force that would bring the wheel's spin in line with
// the ground in one tick, `((spin_velocity_diff * wheel_moment) / tire_radius) / delta`.
// Returns 0 for a non-positive radius or delta.
inline Real needed_rolling_force(Real p_spin_velocity_diff, Real p_wheel_moment, Real p_tire_radius, Real p_delta) {
	if (p_tire_radius <= 0 || p_delta <= 0) {
		return 0;
	}
	return ((p_spin_velocity_diff * p_wheel_moment) / p_tire_radius) / p_delta;
}

// Ported from wheel.gd process_tires(): the longitudinal force cap, the larger of the applied
// torque and the needed rolling force divided by the radius. Returns 0 for a non-positive radius.
inline Real max_longitudinal_force(Real p_applied_torque, Real p_needed_rolling_force, Real p_tire_radius) {
	if (p_tire_radius <= 0) {
		return 0;
	}
	if (std::abs(p_applied_torque) > std::abs(p_needed_rolling_force)) {
		return std::abs(p_applied_torque / p_tire_radius);
	}
	return std::abs(p_needed_rolling_force / p_tire_radius);
}

// Ported from wheel.gd process_tires(): `abs(mass_over_wheel * local_velocity.x) / delta`, the
// force that would stop the lateral motion of the mass over this wheel in one tick.
inline Real max_lateral_force(Real p_mass_over_wheel, Real p_local_velocity_x, Real p_delta) {
	if (p_delta <= 0) {
		return 0;
	}
	return std::abs(p_mass_over_wheel * p_local_velocity_x) / p_delta;
}

// Ported from wheel.gd process_tires(): the brush model proper. Below the critical length the
// contact patch is fully sliding and the friction-limited branch applies; otherwise the linear
// (adhesion) branch. Returns Vec2(lateral x, longitudinal y) in N, before the force caps and
// rolling resistance. Guard added for the port: a (near) zero `1 - slip.y` is replaced by
// kEpsilon in the linear branch.
inline Vec2 brush_tire_force(const Vec2 &p_slip, Real p_cornering_stiffness, Real p_friction, Real p_contact_patch, Real p_longitudinal_grip_ratio, Real p_lateral_grip_assist, Real p_braking_help, Real p_forward_sign) {
	const Real deflect = brush_deflection(p_cornering_stiffness, p_slip);
	Real one_minus_slip_y = static_cast<Real>(1) - p_slip.y;
	const Real crit_length = p_friction * one_minus_slip_y * p_contact_patch * (static_cast<Real>(0.5) * deflect);
	Vec2 force;
	if (crit_length >= p_contact_patch) {
		if (std::abs(one_minus_slip_y) < kEpsilon) {
			one_minus_slip_y = kEpsilon;
		}
		force.y = p_cornering_stiffness * p_slip.y / one_minus_slip_y;
		force.x = p_cornering_stiffness * p_slip.x / one_minus_slip_y;
	} else {
		const Real brushx = (static_cast<Real>(1) - p_friction * one_minus_slip_y * (static_cast<Real>(0.25) * deflect)) * deflect;
		force.y = p_friction * p_longitudinal_grip_ratio * p_cornering_stiffness * p_slip.y * brushx * p_braking_help * p_forward_sign;
		force.x = p_friction * p_cornering_stiffness * p_slip.x * brushx * (std::abs(p_slip.x * p_lateral_grip_assist) + static_cast<Real>(1));
	}
	return force;
}

// Ported from wheel.gd process_tires(): clamps the brush force to the per-tick caps. Sets
// *r_limit_spin exactly as the GDScript's limit_spin flag (true when the longitudinal force
// was capped), used by the vehicle's slip detection.
inline Vec2 limit_tire_force(const Vec2 &p_force, Real p_max_longitudinal_force, Real p_max_lateral_force, bool *r_limit_spin) {
	Vec2 force = p_force;
	bool limited = false;
	if (std::abs(force.y) > std::abs(p_max_longitudinal_force)) {
		force.y = p_max_longitudinal_force * sign(force.y);
		limited = true;
	}
	if (std::abs(force.x) > p_max_lateral_force) {
		force.x = p_max_lateral_force * sign(force.x);
	}
	if (r_limit_spin != nullptr) {
		*r_limit_spin = limited;
	}
	return force;
}

// Ported from wheel.gd process_rolling_resistance():
//     (0.005 + 0.5 * (0.01 + 0.0095 * (v.z * 0.036)^2)) * spring_force * surface_rolling_resistance
// The magnitude only; process_tires() subtracts it times sign(v.z) from force.y.
inline Real rolling_resistance_force(Real p_local_velocity_z, Real p_spring_force, Real p_surface_rolling_resistance) {
	const Real kmh_like = p_local_velocity_z * static_cast<Real>(0.036);
	const Real coefficient = static_cast<Real>(0.005) + (static_cast<Real>(0.5) * (static_cast<Real>(0.01) + (static_cast<Real>(0.0095) * kmh_like * kmh_like)));
	return coefficient * p_spring_force * p_surface_rolling_resistance;
}

// Ported from wheel.gd process_tires() end to end. Inputs are the wheel state and the resolved
// surface parameters (see wheel.gd initialize() for how the surface dictionaries map to them):
//   local_velocity           wheel velocity in the ray's local frame (m/s)
//   spin                     wheel angular velocity (rad/s)
//   tire_radius, wheel_moment, applied_torque, mass_over_wheel, spring_force
//   tire_stiffness           from tire_stiffness(surface_stiffness)
//   contact_patch (m), coefficient_of_friction, tire_width (mm)
//   longitudinal_grip_ratio, lateral_grip_assist, surface_rolling_resistance
//   braking, braking_grip_multiplier, delta (s)
// Returns Vec2(lateral x, longitudinal y) in N including rolling resistance. Optional outputs
// mirror the GDScript members slip_vector, limit_spin and spin_velocity_diff.
inline Vec2 tire_force(const Vec3 &p_local_velocity, Real p_spin, Real p_tire_radius, Real p_wheel_moment, Real p_applied_torque, Real p_mass_over_wheel, Real p_spring_force, Real p_tire_stiffness, Real p_contact_patch, Real p_coefficient_of_friction, Real p_tire_width, Real p_longitudinal_grip_ratio, Real p_lateral_grip_assist, Real p_surface_rolling_resistance, bool p_braking, Real p_braking_grip_multiplier, Real p_delta, Vec2 *r_slip = nullptr, bool *r_limit_spin = nullptr, Real *r_spin_velocity_diff = nullptr) {
	const Real wheel_velocity = wheel_surface_velocity(p_spin, p_tire_radius);
	const Real spin_velocity_diff = spin_velocity_difference(wheel_velocity, p_local_velocity.z);
	const Real needed = needed_rolling_force(spin_velocity_diff, p_wheel_moment, p_tire_radius, p_delta);
	const Real max_y_force = max_longitudinal_force(p_applied_torque, needed, p_tire_radius);
	const Real max_x_force = max_lateral_force(p_mass_over_wheel, p_local_velocity.x, p_delta);
	const Real z_sign = forward_sign(p_local_velocity.z);
	const Vec2 slip = slip_vector(p_local_velocity, p_spin, p_tire_radius);

	const Real stiffness = cornering_stiffness(p_tire_stiffness, p_contact_patch);
	const Real friction = tire_friction(p_coefficient_of_friction, p_spring_force, p_tire_width, p_contact_patch);
	const Real help = braking_help(slip.y, p_braking, p_braking_grip_multiplier);

	Vec2 force = brush_tire_force(slip, stiffness, friction, p_contact_patch, p_longitudinal_grip_ratio, p_lateral_grip_assist, help, z_sign);
	force = limit_tire_force(force, max_y_force, max_x_force, r_limit_spin);
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
