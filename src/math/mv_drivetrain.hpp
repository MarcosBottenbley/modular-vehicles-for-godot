// Modular Vehicles for Godot: motor, gearbox and clutch formulas ported from
// addons/gevp/scripts/vehicle.gd (get_gear_ratio, get_torque_at_rpm, process_motor,
// process_clutch, process_drive).
//
// The torque curve is a Godot Curve resource in the GDScript. Curve is an engine type, so the
// node bakes it into a TorqueTable (kTorqueTableSize samples of the curve over 0..1) and the
// pure sampler below interpolates that table.
//
// Terms:
//   RPM: motor speed in revolutions per minute, the unit of the motor's public API; the rest of
//     the layer uses rad/s.
//   gear index: vehicle.gd's gear numbering: 0 neutral, 1..count the forward gears, -1 reverse.
//   gear ratio (overall): gearbox ratio * final drive, unitless; negative in reverse, 0 in
//     neutral. Multiplies wheel spin to get motor speed and divides torque the other way.
//   final drive: the differential's ratio, unitless.
//   torque table: kTorqueTableSize samples of the torque curve, entry i at normalised RPM
//     i / (kTorqueTableSize - 1); values are unitless torque factors (1 = max torque).
//   torque factor: the curve's value at a normalised RPM, unitless 0..1 for the demo curves.
//   normalised RPM: rpm / max_rpm clamped to 0..1, the torque table's x axis.
//   motor drag torque: the friction torque the motor loses, in N*m, proportional to RPM.
//   torque output: the motor's net torque this tick, in N*m, before the idle/redline cut.
//   throttle amount / clutch amount: smoothed pedal positions, unitless 0..1; clutch 0 is
//     engaged, 1 is fully pressed.
//   motor moment: the motor's moment of inertia, in kg*m^2.
//   gear inertia: the gearbox inertia the motor sees per unit ratio squared, in kg*m^2.
//   drive axles inertia: the sum of the driven wheels' wheel moments, in kg*m^2.
//   drive inertia: everything the motor spins through the gearbox, in kg*m^2.
//   drivetrain spin: the average spin of the driven wheels, in rad/s.
//   wheels reaction torque: the sum of force.y * tire radius over the driven wheels, in N*m,
//     what the road pushes back into the drivetrain.
//   clutch torque: the torque coupling motor and drivetrain this tick, in N*m, limited by the
//     clutch capacity.
//   clutch capacity (max clutch torque): max torque * the owner's clutch ratio, in N*m.
#ifndef MV_DRIVETRAIN_HPP
#define MV_DRIVETRAIN_HPP

#include <array>
#include <cstdint>

#include "mv_math.hpp"

namespace mv {

// vehicle.gd ANGULAR_VELOCITY_TO_RPM := 60.0 / TAU, in RPM per rad/s.
constexpr Real kAngularVelocityToRpm = static_cast<Real>(60) / kTau;

// Number of samples in a baked torque table.
constexpr std::uint32_t kTorqueTableSize = 128;
using TorqueTable = std::array<Real, kTorqueTableSize>;

// rad/s to RPM, as `spin * ANGULAR_VELOCITY_TO_RPM` in the GDScript.
// p_rad_per_s: an angular velocity, in rad/s.
// Returns: the same speed in RPM.
constexpr Real angular_velocity_to_rpm(Real p_rad_per_s) {
	return p_rad_per_s * kAngularVelocityToRpm;
}

// RPM to rad/s, as `rpm / ANGULAR_VELOCITY_TO_RPM` in the GDScript.
// p_rpm: a speed in RPM.
// Returns: the same speed in rad/s.
constexpr Real rpm_to_angular_velocity(Real p_rpm) {
	return p_rpm / kAngularVelocityToRpm;
}

// Overall gear ratio for a gear index.
// Ported from vehicle.gd get_gear_ratio(). Out-of-range indices and a null/empty ratio list
// return 0 instead of indexing out of bounds.
// p_gear_index: the gear index (0 neutral, 1..p_count forward, -1 reverse).
// p_ratios: the forward gearbox ratios, unitless, first gear first; may be null when p_count is 0.
// p_count: how many entries p_ratios holds.
// p_final_drive: the final drive, unitless.
// p_reverse_ratio: the reverse gearbox ratio, unitless, positive.
// Returns: the overall gear ratio, unitless; negative in reverse, 0 in neutral.
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

// Samples a torque table with linear interpolation between its entries. A table filled with
// 1.0 reproduces a null curve (constant factor 1).
// p_table: the torque table.
// p_x: the normalised position 0..1 (clamped), i.e. the normalised RPM.
// Returns: the interpolated torque factor, unitless.
inline Real sample_torque_table(const TorqueTable &p_table, Real p_x) {
	const Real last_index = static_cast<Real>(kTorqueTableSize - 1);
	const Real table_position = clamp(p_x, 0, 1) * last_index;
	const std::uint32_t lower_index = static_cast<std::uint32_t>(table_position);
	if (lower_index >= kTorqueTableSize - 1) {
		return p_table[kTorqueTableSize - 1];
	}
	const Real fraction = table_position - static_cast<Real>(lower_index);
	return lerp(p_table[lower_index], p_table[lower_index + 1], fraction);
}

// Torque factor at a motor speed.
// Ported from vehicle.gd get_torque_at_rpm(): rpm_factor = clamp(rpm / max_rpm, 0, 1), then
// the curve value there.
// p_rpm: the motor speed, in RPM.
// p_max_rpm: the motor's maximum RPM, the curve's right end. Zero or negative returns the
//   table's first entry.
// p_table: the torque table.
// Returns: the torque factor, unitless; torque_at_rpm() scales it to N*m.
inline Real torque_factor_at_rpm(Real p_rpm, Real p_max_rpm, const TorqueTable &p_table) {
	if (p_max_rpm <= 0) {
		return p_table[0];
	}
	return sample_torque_table(p_table, clamp(p_rpm / p_max_rpm, 0, 1));
}

// Motor torque available at a speed.
// Ported from vehicle.gd get_torque_at_rpm(): torque_factor * max_torque. The GDScript does not
// use idle_rpm here (the idle cut-off lives in process_motor()), so neither does this.
// p_rpm: the motor speed, in RPM.
// p_max_rpm: the motor's maximum RPM.
// p_max_torque: the motor's peak torque, in N*m (the curve's value 1).
// p_table: the torque table.
// Returns: the available torque in N*m; motor_torque_output() multiplies it by the throttle.
inline Real torque_at_rpm(Real p_rpm, Real p_max_rpm, Real p_max_torque, const TorqueTable &p_table) {
	return torque_factor_at_rpm(p_rpm, p_max_rpm, p_table) * p_max_torque;
}

// Motor drag torque at a speed.
// Ported from vehicle.gd process_motor(): drag_torque = motor_rpm * motor_drag.
// p_motor_rpm: the motor speed, in RPM.
// p_motor_drag: the owner's drag factor, in N*m per RPM.
// Returns: the motor drag torque in N*m.
inline Real motor_drag_torque(Real p_motor_rpm, Real p_motor_drag) {
	return p_motor_rpm * p_motor_drag;
}

// Net motor torque before the idle/redline cut: throttle share of the curve torque minus the
// drag, which bites harder with the clutch pressed and the throttle closed.
// Ported from vehicle.gd process_motor():
//     curve_torque * throttle - drag_torque * (1 + clutch * (1 - throttle))
// p_curve_torque: from torque_at_rpm(), in N*m.
// p_throttle_amount: the throttle amount, unitless 0..1.
// p_drag_torque: from motor_drag_torque(), in N*m.
// p_clutch_amount: the clutch amount, unitless 0..1.
// Returns: the torque output in N*m; the motor integrates its RPM with it and the clutch
// couples it to the drivetrain.
inline Real motor_torque_output(Real p_curve_torque, Real p_throttle_amount, Real p_drag_torque, Real p_clutch_amount) {
	const Real closed_throttle = static_cast<Real>(1) - p_throttle_amount;
	return p_curve_torque * p_throttle_amount - p_drag_torque * (static_cast<Real>(1) + (p_clutch_amount * closed_throttle));
}

// RPM change for a torque acting on the motor inertia over one tick.
// Ported from vehicle.gd process_motor(): ANGULAR_VELOCITY_TO_RPM * delta * torque / motor_moment.
// p_torque: the torque on the motor, in N*m.
// p_motor_moment: the motor moment, in kg*m^2. Zero or negative returns 0.
// p_delta: the physics tick, in s.
// Returns: the RPM change; the motor adds it to its RPM.
inline Real motor_rpm_delta(Real p_torque, Real p_motor_moment, Real p_delta) {
	if (p_motor_moment <= 0) {
		return 0;
	}
	return kAngularVelocityToRpm * p_delta * p_torque / p_motor_moment;
}

// The inertia the motor sees through the gearbox.
// Ported from vehicle.gd process_clutch(): motor_moment + gear_ratio^2 * gear_inertia +
// drive_axles_inertia. process_drive() uses the same formula with drive_axles_inertia = 0.
// p_motor_moment: the motor moment, in kg*m^2.
// p_gear_ratio: the overall gear ratio, unitless.
// p_gear_inertia: the gear inertia, in kg*m^2.
// p_drive_axles_inertia: the drive axles inertia, in kg*m^2 (0 for the process_drive variant).
// Returns: the drive inertia in kg*m^2, an input of clutch_torque().
inline Real drive_inertia(Real p_motor_moment, Real p_gear_ratio, Real p_gear_inertia, Real p_drive_axles_inertia) {
	const Real ratio_squared = p_gear_ratio * p_gear_ratio;
	return p_motor_moment + (ratio_squared * p_gear_inertia) + p_drive_axles_inertia;
}

// Clutch capacity.
// Ported from vehicle.gd initialize(): max_torque * max_clutch_torque_ratio.
// p_max_torque: the motor's peak torque, in N*m.
// p_max_clutch_torque_ratio: the owner's clutch ratio, unitless (1.6 by default).
// Returns: the clutch capacity in N*m, the clamp of clutch_torque().
inline Real max_clutch_torque(Real p_max_torque, Real p_max_clutch_torque_ratio) {
	return p_max_torque * p_max_clutch_torque_ratio;
}

// The clutch torque that couples motor and drivetrain this tick, clamped to the clutch
// capacity scaled by how far the clutch is engaged. A negative speed difference (motor slower
// than the wheels, overrun) is softened with -sqrt as in the GDScript.
// Ported from vehicle.gd process_clutch().
// p_motor_moment: the motor moment, in kg*m^2.
// p_drive_inertia: from drive_inertia(), in kg*m^2.
// p_gear_ratio: the overall gear ratio, unitless. Neutral (0) returns 0, as the GDScript
//   returns early.
// p_motor_angular_velocity: the motor speed, in rad/s (rpm_to_angular_velocity(motor_rpm)).
// p_drivetrain_spin: the drivetrain spin, in rad/s.
// p_wheels_reaction_torque: the wheels reaction torque, in N*m.
// p_torque_output: from motor_torque_output(), in N*m.
// p_clutch_amount: the clutch amount, unitless 0..1.
// p_max_clutch_torque: the clutch capacity, in N*m.
// p_delta: the physics tick, in s. Zero or negative returns 0.
// Returns: the clutch torque in N*m; the motor loses it and the driven axles receive it times
// the gear ratio.
inline Real clutch_torque(Real p_motor_moment, Real p_drive_inertia, Real p_gear_ratio, Real p_motor_angular_velocity, Real p_drivetrain_spin, Real p_wheels_reaction_torque, Real p_torque_output, Real p_clutch_amount, Real p_max_clutch_torque, Real p_delta) {
	if (is_zero_approx(p_gear_ratio) || p_delta <= 0) {
		return 0;
	}
	// The GDScript's drive_inertia_R: the drive inertia referred to the wheel side.
	const Real drive_inertia_at_wheels = p_drive_inertia / (p_gear_ratio * p_gear_ratio);
	const Real reaction_torque_at_motor = p_wheels_reaction_torque / p_gear_ratio;
	Real speed_difference = p_motor_angular_velocity - (p_drivetrain_spin * p_gear_ratio);
	if (speed_difference < 0) {
		speed_difference = -std::sqrt(-speed_difference);
	}
	// a, b, c are the GDScript's names: the inertial, reaction and motor terms of the coupling.
	const Real a = (p_motor_moment * drive_inertia_at_wheels * speed_difference) / p_delta;
	const Real b = p_motor_moment * reaction_torque_at_motor;
	const Real c = drive_inertia_at_wheels * p_torque_output;
	const Real clutch_factor = static_cast<Real>(1) - p_clutch_amount;
	const Real combined_inertia = p_motor_moment + drive_inertia_at_wheels;
	if (is_zero_approx(combined_inertia)) {
		return 0;
	}
	const Real torque = ((a - b + c) / combined_inertia) * clutch_factor;
	return clamp(torque, -p_max_clutch_torque * clutch_factor, p_max_clutch_torque * clutch_factor);
}

} // namespace mv

#endif // MV_DRIVETRAIN_HPP
