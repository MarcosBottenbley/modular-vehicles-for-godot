#pragma once

// Terms:
//   spring length: full suspension travel along the wheel ray, in m.
//   static load: the weight one wheel carries with the vehicle at rest on flat ground, in N.
//   resting ratio: fraction of spring length compressed under the static load, 0.01..1;
//     1 is fully compressed.
//   spring rate: force per metre of compression, in N/m. Auto mode derives it as
//     static_load / (spring_length * resting_ratio); manual mode takes it as given.
//   damping ratio: damper strength relative to critical damping, unitless; 1 is critical.
//   base damping: the damper rate before the bump and rebound multipliers, in N·s/m. Auto mode
//     derives it from the damping ratio, the spring rate and the static load the way
//     vehicle.gd's calculate_damping() does; manual mode takes it as given.
//   bump: the spring compressing; rebound: the spring extending. Each has its own multiplier
//     on the base damping.
//   fast damping threshold: damper speed above which the fast rate (half the slow rate)
//     applies, in m/s. Was the constant 127 mm/s in wheel.gd.
//   bump stop: the extra force applied when the spring is fully compressed, scaled by a
//     multiplier.
//   anti-roll ratio: anti-roll bar stiffness as a fraction of the spring rate.
//   camber / toe: wheel alignment angles stored as magnitudes in rad; the wheel applies the sign
//     for its side of the vehicle. Camber is not simulated beyond tilting the ray.

#include <godot_cpp/classes/resource.hpp>

namespace godot {

// Spring, damper, anti-roll bar and alignment of a wheel. MVVehicle turns these into per-wheel
// spring and damper rates when it rebuilds, and MVWheel reads the geometry every physics tick.
// An empty slot behaves like the defaults.
class MVSuspensionSettings : public Resource {
	GDCLASS(MVSuspensionSettings, Resource)

public:
	enum SpringRateMode {
		SPRING_RATE_AUTO_FROM_LOAD,
		SPRING_RATE_MANUAL,
	};
	static constexpr int SPRING_RATE_MODE_COUNT = 2;

	static constexpr real_t DEFAULT_SPRING_LENGTH = 0.15; // m
	static constexpr real_t DEFAULT_RESTING_RATIO = 0.5; // unitless
	static constexpr real_t DEFAULT_MANUAL_SPRING_RATE = 49000.0; // N/m, auto mode for a 1500 kg car
	static constexpr real_t DEFAULT_DAMPING_RATIO = 0.4; // unitless
	static constexpr real_t DEFAULT_MANUAL_DAMPING = 3400.0; // N·s/m, auto mode for a 1500 kg car
	static constexpr real_t DEFAULT_BUMP_DAMP_MULTIPLIER = 0.6667; // unitless, 2/3
	static constexpr real_t DEFAULT_REBOUND_DAMP_MULTIPLIER = 1.5; // unitless, 3/2
	static constexpr real_t DEFAULT_FAST_DAMP_THRESHOLD = 0.127; // m/s
	static constexpr real_t DEFAULT_BUMP_STOP_MULTIPLIER = 1.0; // unitless
	static constexpr real_t DEFAULT_ANTI_ROLL_RATIO = 0.25; // unitless
	static constexpr real_t DEFAULT_CAMBER = 0.01745329; // rad, 1 degree
	static constexpr real_t DEFAULT_TOE = 0.01; // rad
	// Spring length and resting ratio divide the static load, so neither may reach 0.
	static constexpr real_t MIN_SPRING_LENGTH = 0.001; // m
	static constexpr real_t MIN_RESTING_RATIO = 0.01; // unitless
	static constexpr real_t MAX_RESTING_RATIO = 1.0; // unitless
	static constexpr real_t MIN_RATE = 0.0; // floor for rates, ratios and multipliers
	static constexpr real_t MAX_ALIGNMENT_ANGLE = Math_PI * 0.25; // rad, +-45 degrees

private:
	real_t spring_length = DEFAULT_SPRING_LENGTH;
	real_t resting_ratio = DEFAULT_RESTING_RATIO;
	SpringRateMode spring_rate_mode = SPRING_RATE_AUTO_FROM_LOAD;
	real_t manual_spring_rate = DEFAULT_MANUAL_SPRING_RATE;
	real_t damping_ratio = DEFAULT_DAMPING_RATIO;
	real_t bump_damp_multiplier = DEFAULT_BUMP_DAMP_MULTIPLIER;
	real_t rebound_damp_multiplier = DEFAULT_REBOUND_DAMP_MULTIPLIER;
	real_t fast_damp_threshold = DEFAULT_FAST_DAMP_THRESHOLD;
	real_t manual_damping = DEFAULT_MANUAL_DAMPING;
	real_t bump_stop_multiplier = DEFAULT_BUMP_STOP_MULTIPLIER;
	real_t anti_roll_ratio = DEFAULT_ANTI_ROLL_RATIO;
	real_t camber = DEFAULT_CAMBER;
	real_t toe = DEFAULT_TOE;

protected:
	static void _bind_methods();
	// Hides the inputs of the inactive spring rate mode so the inspector shows either the
	// auto-mode ratios or the manual rates, never both.
	// p_property: the property the inspector is about to list; its usage flags are edited.
	// Returns: nothing. Only affects the editor; the hidden values are still saved.
	void _validate_property(PropertyInfo &p_property) const;

public:
	// Sets the spring length, clamped to >= MIN_SPRING_LENGTH.
	// p_spring_length: suspension travel in m.
	// Returns: nothing. Emits changed; MVVehicle recomputes spring rates on its next rebuild.
	void set_spring_length(real_t p_spring_length);
	// Returns: the spring length in m.
	real_t get_spring_length() const;

	// Sets the resting ratio, clamped to MIN_RESTING_RATIO..MAX_RESTING_RATIO.
	// p_resting_ratio: fraction of spring length compressed under the static load, unitless.
	// Returns: nothing. Emits changed; MVVehicle recomputes spring rates in auto mode.
	void set_resting_ratio(real_t p_resting_ratio);
	// Returns: the resting ratio, unitless.
	real_t get_resting_ratio() const;

	// Sets the spring rate mode. Out-of-range values are rejected with an error.
	// p_spring_rate_mode: SPRING_RATE_AUTO_FROM_LOAD or SPRING_RATE_MANUAL.
	// Returns: nothing. Emits changed and refreshes the inspector's property list.
	void set_spring_rate_mode(SpringRateMode p_spring_rate_mode);
	// Returns: the spring rate mode.
	SpringRateMode get_spring_rate_mode() const;

	// Sets the manual spring rate, clamped to >= MIN_RATE.
	// p_manual_spring_rate: spring rate in N/m, used only in SPRING_RATE_MANUAL.
	// Returns: nothing. Emits changed; MVVehicle uses it on its next rebuild in manual mode.
	void set_manual_spring_rate(real_t p_manual_spring_rate);
	// Returns: the manual spring rate in N/m.
	real_t get_manual_spring_rate() const;

	// Sets the damping ratio, clamped to >= MIN_RATE.
	// p_damping_ratio: damper strength relative to critical damping, unitless.
	// Returns: nothing. Emits changed; MVVehicle recomputes the base damping in auto mode.
	void set_damping_ratio(real_t p_damping_ratio);
	// Returns: the damping ratio, unitless.
	real_t get_damping_ratio() const;

	// Sets the bump damping multiplier, clamped to >= MIN_RATE.
	// p_bump_damp_multiplier: factor on the base damping during bump, unitless.
	// Returns: nothing. Emits changed; MVWheel applies the new bump rate next tick.
	void set_bump_damp_multiplier(real_t p_bump_damp_multiplier);
	// Returns: the bump damping multiplier, unitless.
	real_t get_bump_damp_multiplier() const;

	// Sets the rebound damping multiplier, clamped to >= MIN_RATE.
	// p_rebound_damp_multiplier: factor on the base damping during rebound, unitless.
	// Returns: nothing. Emits changed; MVWheel applies the new rebound rate next tick.
	void set_rebound_damp_multiplier(real_t p_rebound_damp_multiplier);
	// Returns: the rebound damping multiplier, unitless.
	real_t get_rebound_damp_multiplier() const;

	// Sets the fast damping threshold, clamped to >= MIN_RATE.
	// p_fast_damp_threshold: damper speed in m/s above which the fast rate applies.
	// Returns: nothing. Emits changed; MVWheel reads it in its damper step next tick.
	void set_fast_damp_threshold(real_t p_fast_damp_threshold);
	// Returns: the fast damping threshold in m/s.
	real_t get_fast_damp_threshold() const;

	// Sets the manual base damping, clamped to >= MIN_RATE.
	// p_manual_damping: base damping in N·s/m, used only in SPRING_RATE_MANUAL.
	// Returns: nothing. Emits changed; MVVehicle uses it on its next rebuild in manual mode.
	void set_manual_damping(real_t p_manual_damping);
	// Returns: the manual base damping in N·s/m.
	real_t get_manual_damping() const;

	// Sets the bump stop multiplier, clamped to >= MIN_RATE.
	// p_bump_stop_multiplier: factor on the bump stop force, unitless.
	// Returns: nothing. Emits changed; MVWheel reads it when the spring bottoms out.
	void set_bump_stop_multiplier(real_t p_bump_stop_multiplier);
	// Returns: the bump stop multiplier, unitless.
	real_t get_bump_stop_multiplier() const;

	// Sets the anti-roll ratio, clamped to >= MIN_RATE.
	// p_anti_roll_ratio: anti-roll bar stiffness as a fraction of the spring rate, unitless.
	// Returns: nothing. Emits changed; MVVehicle recomputes the anti-roll stiffness on rebuild.
	void set_anti_roll_ratio(real_t p_anti_roll_ratio);
	// Returns: the anti-roll ratio, unitless.
	real_t get_anti_roll_ratio() const;

	// Sets the camber magnitude, clamped to +-MAX_ALIGNMENT_ANGLE.
	// p_camber: camber angle in rad; the inspector edits it in degrees.
	// Returns: nothing. Emits changed; MVWheel re-tilts its ray on its next physics tick.
	void set_camber(real_t p_camber);
	// Returns: the camber magnitude in rad.
	real_t get_camber() const;

	// Sets the toe magnitude, clamped to +-MAX_ALIGNMENT_ANGLE.
	// p_toe: toe angle in rad; the inspector edits it in degrees.
	// Returns: nothing. Emits changed; MVWheel adds it to its steering angle next tick.
	void set_toe(real_t p_toe);
	// Returns: the toe magnitude in rad.
	real_t get_toe() const;
};

} // namespace godot

VARIANT_ENUM_CAST(MVSuspensionSettings::SpringRateMode);
