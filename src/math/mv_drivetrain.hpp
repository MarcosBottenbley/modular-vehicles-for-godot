// Modular Vehicles for Godot: motor, gearbox and clutch formulas ported from
// addons/gevp/scripts/vehicle.gd (get_gear_ratio, get_torque_at_rpm, process_motor,
// process_clutch, process_drive).
//
// The torque curve is a Godot Curve resource in the GDScript. Curve is an engine type, so the
// node bakes it into a TorqueTable (kTorqueTableSize samples of the curve over 0..1) and the
// pure sampler below interpolates that table.
#ifndef MV_DRIVETRAIN_HPP
#define MV_DRIVETRAIN_HPP

#include <array>
#include <cstdint>

#include "mv_math.hpp"

namespace mv {

// vehicle.gd ANGULAR_VELOCITY_TO_RPM := 60.0 / TAU
constexpr Real kAngularVelocityToRpm = static_cast<Real>(60) / kTau;

constexpr std::uint32_t kTorqueTableSize = 128;
using TorqueTable = std::array<Real, kTorqueTableSize>;

// rad/s -> RPM and back, as `spin * ANGULAR_VELOCITY_TO_RPM` / `rpm / ANGULAR_VELOCITY_TO_RPM`.
constexpr Real angular_velocity_to_rpm(Real p_rad_per_s) {
	return p_rad_per_s * kAngularVelocityToRpm;
}
constexpr Real rpm_to_angular_velocity(Real p_rpm) {
	return p_rpm / kAngularVelocityToRpm;
}

// Ported from vehicle.gd get_gear_ratio(). Same indexing: 0 is neutral, 1..count are the
// forward gears (ratios[gear - 1] * final_drive), -1 is reverse (-reverse_ratio * final_drive).
// Out-of-range indices and a null/empty ratio list return 0 instead of indexing out of bounds.
inline Real gear_ratio(int p_gear_index, const Real *p_ratios, std::uint32_t p_count, Real p_final_drive, Real p_reverse_ratio) {
	if (p_gear_index > 0) {
		if (p_ratios == nullptr || static_cast<std::uint32_t>(p_gear_index) > p_count) {
			return 0;
		}
		return p_ratios[p_gear_index - 1] * p_final_drive;
	}
	if (p_gear_index == -1) {
		return -p_reverse_ratio * p_final_drive;
	}
	return 0;
}

// Samples a baked torque table at a normalised position (0..1) with linear interpolation.
// Entry i holds the curve value at x = i / (kTorqueTableSize - 1). A table filled with 1.0
// reproduces a null curve (constant factor 1).
inline Real sample_torque_table(const TorqueTable &p_table, Real p_x) {
	const Real x = clamp(p_x, 0, 1) * static_cast<Real>(kTorqueTableSize - 1);
	std::uint32_t i = static_cast<std::uint32_t>(x);
	if (i >= kTorqueTableSize - 1) {
		return p_table[kTorqueTableSize - 1];
	}
	const Real t = x - static_cast<Real>(i);
	return lerp(p_table[i], p_table[i + 1], t);
}

// Ported from vehicle.gd get_torque_at_rpm(): `rpm_factor = clamp(rpm / max_rpm, 0, 1)` then
// the curve value at rpm_factor. Returns the dimensionless torque factor (0..1 for the demo
// curves). A non-positive max_rpm returns the table's first entry.
inline Real torque_factor_at_rpm(Real p_rpm, Real p_max_rpm, const TorqueTable &p_table) {
	if (p_max_rpm <= 0) {
		return p_table[0];
	}
	return sample_torque_table(p_table, clamp(p_rpm / p_max_rpm, 0, 1));
}

// Ported from vehicle.gd get_torque_at_rpm(): `torque_factor * max_torque` in N*m. The GDScript
// does not use idle_rpm here (the idle cut-off lives in process_motor), so neither does this.
inline Real torque_at_rpm(Real p_rpm, Real p_max_rpm, Real p_max_torque, const TorqueTable &p_table) {
	return torque_factor_at_rpm(p_rpm, p_max_rpm, p_table) * p_max_torque;
}

// Ported from vehicle.gd process_motor(): `drag_torque = motor_rpm * motor_drag`.
inline Real motor_drag_torque(Real p_motor_rpm, Real p_motor_drag) {
	return p_motor_rpm * p_motor_drag;
}

// Ported from vehicle.gd process_motor(): the net motor torque before the idle/redline cut,
// `curve_torque * throttle - drag_torque * (1 + clutch * (1 - throttle))`.
inline Real motor_torque_output(Real p_curve_torque, Real p_throttle_amount, Real p_drag_torque, Real p_clutch_amount) {
	return p_curve_torque * p_throttle_amount - p_drag_torque * (static_cast<Real>(1) + (p_clutch_amount * (static_cast<Real>(1) - p_throttle_amount)));
}

// Ported from vehicle.gd process_motor(): RPM change for a torque acting on the motor inertia
// over one tick, `ANGULAR_VELOCITY_TO_RPM * delta * torque / motor_moment`.
inline Real motor_rpm_delta(Real p_torque, Real p_motor_moment, Real p_delta) {
	if (p_motor_moment <= 0) {
		return 0;
	}
	return kAngularVelocityToRpm * p_delta * p_torque / p_motor_moment;
}

// Ported from vehicle.gd process_clutch(): the inertia the motor sees through the gearbox,
// `motor_moment + gear_ratio^2 * gear_inertia + drive_axles_inertia`. process_drive() uses the
// same formula with drive_axles_inertia = 0.
inline Real drive_inertia(Real p_motor_moment, Real p_gear_ratio, Real p_gear_inertia, Real p_drive_axles_inertia) {
	return p_motor_moment + (p_gear_ratio * p_gear_ratio * p_gear_inertia) + p_drive_axles_inertia;
}

// Ported from vehicle.gd initialize(): `max_torque * max_clutch_torque_ratio`.
inline Real max_clutch_torque(Real p_max_torque, Real p_max_clutch_torque_ratio) {
	return p_max_torque * p_max_clutch_torque_ratio;
}

// Ported from vehicle.gd process_clutch(): the torque that couples motor and drivetrain this
// tick, clamped to +-max_clutch_torque scaled by how far the clutch is engaged.
//   motor_angular_velocity      motor_rpm / ANGULAR_VELOCITY_TO_RPM (rad/s)
//   drivetrain_spin             average spin of the drive wheels (rad/s)
//   wheels_reaction_torque      sum of force_vector.y * tire_radius over the drive wheels
//   clutch_amount               0 = engaged, 1 = pedal fully pressed
// Returns 0 in neutral (gear_ratio == 0, the GDScript returns early) or for a non-positive delta.
inline Real clutch_torque(Real p_motor_moment, Real p_drive_inertia, Real p_gear_ratio, Real p_motor_angular_velocity, Real p_drivetrain_spin, Real p_wheels_reaction_torque, Real p_torque_output, Real p_clutch_amount, Real p_max_clutch_torque, Real p_delta) {
	if (is_zero_approx(p_gear_ratio) || p_delta <= 0) {
		return 0;
	}
	const Real drive_inertia_r = p_drive_inertia / (p_gear_ratio * p_gear_ratio);
	const Real reaction_torque = p_wheels_reaction_torque / p_gear_ratio;
	Real speed_difference = p_motor_angular_velocity - (p_drivetrain_spin * p_gear_ratio);
	if (speed_difference < 0) {
		speed_difference = -std::sqrt(-speed_difference);
	}
	const Real a = (p_motor_moment * drive_inertia_r * speed_difference) / p_delta;
	const Real b = p_motor_moment * reaction_torque;
	const Real c = drive_inertia_r * p_torque_output;
	const Real clutch_factor = static_cast<Real>(1) - p_clutch_amount;
	const Real denominator = p_motor_moment + drive_inertia_r;
	if (is_zero_approx(denominator)) {
		return 0;
	}
	const Real torque = ((a - b + c) / denominator) * clutch_factor;
	return clamp(torque, -p_max_clutch_torque * clutch_factor, p_max_clutch_torque * clutch_factor);
}

} // namespace mv

#endif // MV_DRIVETRAIN_HPP
