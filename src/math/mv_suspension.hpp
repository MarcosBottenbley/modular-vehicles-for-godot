// Modular Vehicles for Godot: suspension formulas ported 1:1 from addons/gevp/scripts/vehicle.gd
// (calculate_spring_rate, calculate_damping, calculate_axle_spring_force) and
// addons/gevp/scripts/wheel.gd (process_suspension).
//
// Units follow the GDScript: compression and spring speed are in millimetres and mm/s, so a
// spring rate is N/mm and a damping coefficient is N per mm/s. Loads are per wheel in N.
//
// Terms:
//   static load (wheel load): the weight one wheel carries with the vehicle at rest, in N.
//     vehicle.gd passes `vehicle_mass * weight_distribution * 4.9`, i.e. the load on one wheel
//     of a two-wheel axle (mass * distribution * 9.8 / 2).
//   spring length: the full suspension travel of the wheel, in m.
//   resting ratio: the fraction of the spring length compressed under the static load, 0..1,
//     unitless. 0.5 means the wheel sits halfway through its travel at rest.
//   compression: how far the spring is shorter than its full length, in mm (the GDScript
//     multiplies metres by 1000 here and keeps mm for the rest of the suspension maths).
//   spring speed: the rate of change of compression, in mm/s. Positive is bump (compressing),
//     negative is rebound (extending).
//   spring rate: force per unit compression, in N/mm.
//   damping ratio: unitless fraction of critical damping the owner tunes (0.4 in the demo).
//   damping coefficient: force per unit spring speed, in N per mm/s, derived from the damping
//     ratio by damping_coefficient().
//   slow / fast damping: the damping coefficients below and above the fast-damp threshold.
//   fast-damp threshold: the spring speed, in mm/s, above which the fast coefficients apply to
//     the excess speed (127 mm/s in wheel.gd).
//   bump stop: the extra force applied while the suspension is bottomed out (compression at
//     its maximum) to keep the body off the ground, in N.
//   antiroll force: the force an anti-roll bar adds from the compression difference with the
//     opposite wheel, in N; antiroll is its stiffness in N/mm.
//   mass over wheel: the share of the vehicle mass resting on this wheel, in kg.
//   total suspension force: spring + antiroll + damper + bump stop, floored at zero, in N; it
//     is what the wheel pushes along its ray.
#ifndef MV_SUSPENSION_HPP
#define MV_SUSPENSION_HPP

#include "mv_math.hpp"

namespace mv {

// Metres to millimetres, the unit change wheel.gd makes for compression (unitless factor).
constexpr Real kMetresToMillimetres = static_cast<Real>(1000);
// Millimetres to metres, the inverse change the GDScript writes as `* 0.001`.
constexpr Real kMillimetresToMetres = static_cast<Real>(0.001);
// vehicle.gd calculate_damping() scales the textbook critical damping by this (unitless).
constexpr Real kDampingUnitScale = static_cast<Real>(0.01);
// wheel.gd clamps the bump-stop spring speed to this many m/s before using it.
constexpr Real kBumpStopMaxSpeed = static_cast<Real>(5);
// vehicle.gd calculate_axle_spring_force() counts both wheels of an axle (unitless).
constexpr Real kWheelsPerAxle = static_cast<Real>(2);

// Spring rate that holds the static load at the resting compression.
// Ported from vehicle.gd calculate_spring_rate().
// p_wheel_load: the static load this wheel carries, in N.
// p_spring_length: the spring length, in m. Zero or negative returns 0.
// p_resting_ratio: the resting ratio, unitless 0..1. Zero or negative returns 0.
// Returns: the spring rate in N/mm; the vehicle caches it per wheel and spring_force() uses it.
inline Real spring_rate(Real p_wheel_load, Real p_spring_length, Real p_resting_ratio) {
	if (p_spring_length <= kEpsilon || p_resting_ratio <= kEpsilon) {
		return 0;
	}
	// The GDScript writes the ratio as (spring_length * resting_ratio) / spring_length; kept
	// as written so the rounding matches the golden values.
	const Real corrected_resting_ratio = (p_spring_length * p_resting_ratio) / p_spring_length;
	const Real target_compression = p_spring_length * corrected_resting_ratio * kMetresToMillimetres;
	return p_wheel_load / target_compression;
}

// Damping coefficient for a damping ratio, from the critical damping of the spring and load.
// Ported from vehicle.gd calculate_damping(): damping_ratio * 2 * sqrt(rate * load) * 0.01.
// p_spring_rate: the spring rate, in N/mm.
// p_wheel_load: the static load, in N.
// p_damping_ratio: the damping ratio, unitless.
// Returns: the damping coefficient in N per mm/s; the vehicle derives the slow and fast bump
// and rebound coefficients from it. 0 when the rate * load product is not positive.
inline Real damping_coefficient(Real p_spring_rate, Real p_wheel_load, Real p_damping_ratio) {
	const Real rate_times_load = p_spring_rate * p_wheel_load;
	if (rate_times_load <= 0) {
		return 0;
	}
	return p_damping_ratio * static_cast<Real>(2) * std::sqrt(rate_times_load) * kDampingUnitScale;
}

// Compression of the spring from its current length.
// Ported from wheel.gd process_suspension(): (spring_length - spring_current_length) * 1000.
// p_spring_length: the spring length, in m.
// p_spring_current_length: the ray hit distance minus the tire radius, in m.
// Returns: the compression in mm, the input of spring_force() and antiroll_force().
inline Real compression_mm(Real p_spring_length, Real p_spring_current_length) {
	return (p_spring_length - p_spring_current_length) * kMetresToMillimetres;
}

// Hooke's law in the GDScript's units.
// Ported from wheel.gd process_suspension(): spring_force = compression * spring_rate.
// p_spring_rate: the spring rate, in N/mm.
// p_compression_mm: the compression, in mm.
// Returns: the spring force in N, one term of total_suspension_force().
inline Real spring_force(Real p_spring_rate, Real p_compression_mm) {
	return p_compression_mm * p_spring_rate;
}

// Anti-roll bar force from the compression difference across an axle.
// Ported from wheel.gd process_suspension(): antiroll * (compression - opposite_compression).
// p_antiroll: the bar stiffness, in N/mm (vehicle.gd uses spring_rate * arb_ratio).
// p_compression_mm: this wheel's compression, in mm.
// p_opposite_compression_mm: the opposite wheel's compression on the same axle, in mm.
// Returns: the antiroll force in N, positive when this side is more compressed.
inline Real antiroll_force(Real p_antiroll, Real p_compression_mm, Real p_opposite_compression_mm) {
	return p_antiroll * (p_compression_mm - p_opposite_compression_mm);
}

// Damper force with separate slow and fast coefficients for bump and rebound.
// Ported from wheel.gd process_suspension() (the damping_force block). Above the fast-damp
// threshold the fast coefficient applies to the excess speed only. When bottomed out the
// GDScript cancels the bump coefficients (bottom_out_damping = -slow_bump,
// bottom_out_damping_fast = -fast_bump); rebound is unaffected.
// p_spring_speed_mm_s: the spring speed, in mm/s; positive is bump.
// p_slow_bump, p_fast_bump: the bump damping coefficients, in N per mm/s.
// p_slow_rebound, p_fast_rebound: the rebound damping coefficients, in N per mm/s.
// p_fast_damp_threshold: the fast-damp threshold, in mm/s.
// p_bottomed_out: true while the compression is at its maximum this tick.
// Returns: the damper force in N (positive in bump, negative in rebound), one term of
// total_suspension_force().
inline Real damper_force(Real p_spring_speed_mm_s, Real p_slow_bump, Real p_fast_bump, Real p_slow_rebound, Real p_fast_rebound, Real p_fast_damp_threshold, bool p_bottomed_out) {
	const Real bottom_out_damping = p_bottomed_out ? -p_slow_bump : static_cast<Real>(0);
	const Real bottom_out_damping_fast = p_bottomed_out ? -p_fast_bump : static_cast<Real>(0);
	if (p_spring_speed_mm_s >= 0) {
		if (p_spring_speed_mm_s > p_fast_damp_threshold) {
			const Real excess_speed = p_spring_speed_mm_s - p_fast_damp_threshold;
			return (excess_speed * (p_fast_bump + bottom_out_damping_fast)) + (p_fast_damp_threshold * (p_slow_bump + bottom_out_damping));
		}
		return p_spring_speed_mm_s * (p_slow_bump + bottom_out_damping);
	}
	if (p_spring_speed_mm_s < -p_fast_damp_threshold) {
		const Real excess_speed = p_spring_speed_mm_s + p_fast_damp_threshold;
		return (excess_speed * p_fast_rebound) + (-p_fast_damp_threshold * p_slow_rebound);
	}
	return p_spring_speed_mm_s * p_slow_rebound;
}

// Bump stop force while the suspension is bottomed out: the force to stop the mass over the
// wheel within one tick plus the force to hold it against gravity.
// Ported from wheel.gd process_suspension() (the bottom_out_force term).
// p_mass_over_wheel: the mass over wheel, in kg.
// p_spring_speed_mm_s: the spring speed, in mm/s; only bump speed (positive) counts, clamped
//   to kBumpStopMaxSpeed after conversion to m/s.
// p_delta: the physics tick, in s. Zero or negative returns 0.
// p_gravity_magnitude: |gravity|, in m/s^2.
// p_gravity_on_spring: dot of the wheel's up axis with the up direction, clamped to 0..1,
//   unitless; 1 on flat ground.
// p_bump_stop_multiplier: the owner's tuning factor, unitless (1 = as computed).
// Returns: the bump stop force in N, one term of total_suspension_force(); 0 unless bottomed out
// (callers only ask while bottomed out).
inline Real bump_stop_force(Real p_mass_over_wheel, Real p_spring_speed_mm_s, Real p_delta, Real p_gravity_magnitude, Real p_gravity_on_spring, Real p_bump_stop_multiplier) {
	if (p_delta <= 0) {
		return 0;
	}
	const Real clamped_speed_m_s = clamp(p_spring_speed_mm_s * kMillimetresToMetres, 0, kBumpStopMaxSpeed);
	const Real stopping_force = (p_mass_over_wheel * clamped_speed_m_s) / p_delta;
	const Real holding_force = p_mass_over_wheel * p_gravity_magnitude * clamp(p_gravity_on_spring, 0, 1);
	return (stopping_force + holding_force) * p_bump_stop_multiplier;
}

// The total suspension force the wheel pushes along its ray.
// Ported from wheel.gd process_suspension(): maxf(0, spring + antiroll + damping + bottom_out).
// p_spring_force: from spring_force(), in N.
// p_antiroll_force: from antiroll_force(), in N.
// p_damper_force: from damper_force(), in N.
// p_bump_stop_force: from bump_stop_force(), in N (0 when not bottomed out).
// Returns: the total suspension force in N, never negative; the vehicle applies it along the
// contact normal and the tire model uses it as the normal force.
inline Real total_suspension_force(Real p_spring_force, Real p_antiroll_force, Real p_damper_force, Real p_bump_stop_force) {
	return std::max(static_cast<Real>(0), p_spring_force + p_antiroll_force + p_damper_force + p_bump_stop_force);
}

// Next tick's maximum spring length: the wheel may only drop as fast as the spring can push it,
// which keeps an unloaded wheel from snapping to full extension in one tick.
// Ported from wheel.gd process_suspension() (the max_spring_length update).
// p_total_force: this tick's total suspension force, in N.
// p_wheel_mass: the wheel's mass, in kg. Zero or negative keeps the current length.
// p_spring_speed_mm_s: the spring speed, in mm/s.
// p_delta: the physics tick, in s.
// p_spring_current_length: the spring's current length, in m.
// p_spring_length: the spring length, in m, the upper clamp.
// Returns: the maximum length in m for the next tick, clamped to 0..p_spring_length; the wheel
// clamps its measured length to it.
inline Real next_max_spring_length(Real p_total_force, Real p_wheel_mass, Real p_spring_speed_mm_s, Real p_delta, Real p_spring_current_length, Real p_spring_length) {
	if (p_wheel_mass <= 0) {
		return clamp(p_spring_current_length, 0, p_spring_length);
	}
	// (force / mass - speed) is an acceleration in the GDScript's mixed units; times delta and
	// the mm->m factor it becomes the length change allowed this tick.
	const Real allowed_change = ((p_total_force / p_wheel_mass) - p_spring_speed_mm_s) * p_delta * kMillimetresToMetres;
	return clamp(allowed_change + p_spring_current_length, 0, p_spring_length);
}

// Spring force of a whole axle at a given compression fraction, used by vehicle.gd to derive
// the default front brake bias from the front and rear axles.
// Ported from vehicle.gd calculate_axle_spring_force().
// p_compression_ratio: the fraction of the spring length compressed, unitless 0..1.
// p_spring_length: the spring length, in m.
// p_spring_rate: the spring rate, in N/mm.
// Returns: the axle's spring force in N (two wheels).
inline Real axle_spring_force(Real p_compression_ratio, Real p_spring_length, Real p_spring_rate) {
	return p_spring_length * p_compression_ratio * kMetresToMillimetres * p_spring_rate * kWheelsPerAxle;
}

} // namespace mv

#endif // MV_SUSPENSION_HPP
