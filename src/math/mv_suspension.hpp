// Modular Vehicles for Godot: suspension formulas ported 1:1 from addons/gevp/scripts/vehicle.gd
// (calculate_spring_rate, calculate_damping, calculate_axle_spring_force) and
// addons/gevp/scripts/wheel.gd (process_suspension).
//
// Units follow the GDScript: compression and spring speed are in millimetres and mm/s, so
// spring_rate is N/mm and the damping coefficients are N per mm/s. Loads are per wheel in N:
// vehicle.gd passes `vehicle_mass * weight_distribution * 4.9`, i.e. the static load on one
// wheel of a two-wheel axle (mass * distribution * 9.8 / 2).
#ifndef MV_SUSPENSION_HPP
#define MV_SUSPENSION_HPP

#include "mv_math.hpp"

namespace mv {

// Ported from vehicle.gd calculate_spring_rate(). Returns N/mm. Zero when spring_length or
// resting_ratio is (near) zero instead of dividing by zero.
inline Real spring_rate(Real p_wheel_load, Real p_spring_length, Real p_resting_ratio) {
	if (p_spring_length <= kEpsilon || p_resting_ratio <= kEpsilon) {
		return 0;
	}
	// corrected_resting_ratio = (spring_length * resting_ratio) / spring_length
	const Real corrected_resting_ratio = (p_spring_length * p_resting_ratio) / p_spring_length;
	const Real target_compression = p_spring_length * corrected_resting_ratio * static_cast<Real>(1000);
	return p_wheel_load / target_compression;
}

// Ported from vehicle.gd calculate_damping(). Returns N per mm/s.
inline Real damping_coefficient(Real p_spring_rate, Real p_wheel_load, Real p_damping_ratio) {
	const Real product = p_spring_rate * p_wheel_load;
	if (product <= 0) {
		return 0;
	}
	return p_damping_ratio * static_cast<Real>(2) * std::sqrt(product) * static_cast<Real>(0.01);
}

// Ported from wheel.gd process_suspension(): `(spring_length - spring_current_length) * 1000.0`.
inline Real compression_mm(Real p_spring_length, Real p_spring_current_length) {
	return (p_spring_length - p_spring_current_length) * static_cast<Real>(1000);
}

// Ported from wheel.gd process_suspension(): `spring_force = compression * spring_rate`.
inline Real spring_force(Real p_spring_rate, Real p_compression_mm) {
	return p_compression_mm * p_spring_rate;
}

// Ported from wheel.gd process_suspension(): `antiroll * (compression - opposite_compression)`.
inline Real antiroll_force(Real p_antiroll, Real p_compression_mm, Real p_opposite_compression_mm) {
	return p_antiroll * (p_compression_mm - p_opposite_compression_mm);
}

// Ported from wheel.gd process_suspension() (the damping_force block). Positive spring speed
// is bump (compressing), negative is rebound. Above fast_damp_threshold (mm/s) the fast
// coefficient applies to the excess speed only. When the suspension is bottomed out the
// GDScript cancels the bump coefficients (bottom_out_damping = -slow_bump,
// bottom_out_damping_fast = -fast_bump); rebound is unaffected.
inline Real damper_force(Real p_spring_speed_mm_s, Real p_slow_bump, Real p_fast_bump, Real p_slow_rebound, Real p_fast_rebound, Real p_fast_damp_threshold, bool p_bottomed_out) {
	const Real bottom_out_damping = p_bottomed_out ? -p_slow_bump : static_cast<Real>(0);
	const Real bottom_out_damping_fast = p_bottomed_out ? -p_fast_bump : static_cast<Real>(0);
	if (p_spring_speed_mm_s >= 0) {
		if (p_spring_speed_mm_s > p_fast_damp_threshold) {
			return ((p_spring_speed_mm_s - p_fast_damp_threshold) * (p_fast_bump + bottom_out_damping_fast)) + (p_fast_damp_threshold * (p_slow_bump + bottom_out_damping));
		}
		return p_spring_speed_mm_s * (p_slow_bump + bottom_out_damping);
	}
	if (p_spring_speed_mm_s < -p_fast_damp_threshold) {
		return ((p_spring_speed_mm_s + p_fast_damp_threshold) * p_fast_rebound) + (-p_fast_damp_threshold * p_slow_rebound);
	}
	return p_spring_speed_mm_s * p_slow_rebound;
}

// Ported from wheel.gd process_suspension() (the bottom_out_force term). gravity_on_spring is
// the clamped dot product of the wheel's up axis with the normalised up direction (0..1).
// Returns zero for a non-positive delta instead of dividing by zero.
inline Real bump_stop_force(Real p_mass_over_wheel, Real p_spring_speed_mm_s, Real p_delta, Real p_gravity_magnitude, Real p_gravity_on_spring, Real p_bump_stop_multiplier) {
	if (p_delta <= 0) {
		return 0;
	}
	const Real speed_term = (p_mass_over_wheel * clamp(p_spring_speed_mm_s * static_cast<Real>(0.001), 0, 5)) / p_delta;
	const Real gravity_term = p_mass_over_wheel * p_gravity_magnitude * clamp(p_gravity_on_spring, 0, 1);
	return (speed_term + gravity_term) * p_bump_stop_multiplier;
}

// Ported from wheel.gd process_suspension(): the total force pushed through the ray,
// `maxf(0, spring + antiroll + damping + bottom_out)`.
inline Real total_suspension_force(Real p_spring_force, Real p_antiroll_force, Real p_damper_force, Real p_bump_stop_force) {
	return std::max(static_cast<Real>(0), p_spring_force + p_antiroll_force + p_damper_force + p_bump_stop_force);
}

// Ported from wheel.gd process_suspension(): the next tick's max_spring_length, which lets the
// wheel drop at most as fast as the spring can push it.
inline Real next_max_spring_length(Real p_total_force, Real p_wheel_mass, Real p_spring_speed_mm_s, Real p_delta, Real p_spring_current_length, Real p_spring_length) {
	if (p_wheel_mass <= 0) {
		return clamp(p_spring_current_length, 0, p_spring_length);
	}
	return clamp((((p_total_force / p_wheel_mass) - p_spring_speed_mm_s) * p_delta * static_cast<Real>(0.001)) + p_spring_current_length, 0, p_spring_length);
}

// Ported from vehicle.gd calculate_axle_spring_force(); used to derive the default brake bias.
inline Real axle_spring_force(Real p_compression_ratio, Real p_spring_length, Real p_spring_rate) {
	return p_spring_length * p_compression_ratio * static_cast<Real>(1000) * p_spring_rate * static_cast<Real>(2);
}

} // namespace mv

#endif // MV_SUSPENSION_HPP
