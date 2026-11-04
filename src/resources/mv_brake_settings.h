#pragma once

// Terms:
//   ABS: anti-lock braking. When a braked wheel spins much slower than the ground moves under
//     it, the brake on that wheel is released for a short pulse so the tire keeps rolling.
//   pulse time: how long one ABS release lasts, in s.
//   spin difference: wheel rim speed minus ground speed along the wheel, in m/s. Stored as a
//     positive magnitude; the wheel compares against its negative like wheel.gd did.
//   static load: the weight one wheel carries with the vehicle at rest on flat ground, in N.
//   bias weight: this wheel's share of the vehicle's total brake force. -1 means "derive it
//     from the static load"; 0 or more is a relative weight normalised over all wheels.

#include <godot_cpp/classes/resource.hpp>

namespace godot {

// Anti-lock braking and brake force share of a wheel. MVVehicle normalises the bias weights
// of all wheels when it rebuilds, and MVWheel runs the ABS logic every physics tick. An empty
// slot behaves like the defaults, which match the always-on ABS of the GDScript.
class MVBrakeSettings : public Resource {
	GDCLASS(MVBrakeSettings, Resource)

public:
	static constexpr bool DEFAULT_ABS_ENABLED = true;
	static constexpr real_t DEFAULT_ABS_PULSE_TIME = 0.03; // s
	static constexpr real_t DEFAULT_ABS_SPIN_DIFFERENCE_THRESHOLD = 12.0; // m/s
	static constexpr real_t AUTOMATIC_BIAS_WEIGHT = -1.0; // unitless sentinel, also the default
	static constexpr real_t MIN_ABS_VALUE = 0.0; // floor for the pulse time and the threshold

private:
	bool abs_enabled = DEFAULT_ABS_ENABLED;
	real_t abs_pulse_time = DEFAULT_ABS_PULSE_TIME;
	real_t abs_spin_difference_threshold = DEFAULT_ABS_SPIN_DIFFERENCE_THRESHOLD;
	real_t bias_weight = AUTOMATIC_BIAS_WEIGHT;

protected:
	static void _bind_methods();

public:
	// Enables or disables ABS on this wheel.
	// p_abs_enabled: true lets the wheel release its brake in pulses.
	// Returns: nothing. Emits changed; MVWheel checks the flag before each ABS decision.
	void set_abs_enabled(bool p_abs_enabled);
	// Returns: true when ABS may release the brake on this wheel.
	bool is_abs_enabled() const;

	// Sets the pulse time, clamped to >= MIN_ABS_VALUE.
	// p_abs_pulse_time: length of one ABS release in s.
	// Returns: nothing. Emits changed; MVWheel uses it for the next pulse it starts.
	void set_abs_pulse_time(real_t p_abs_pulse_time);
	// Returns: the pulse time in s.
	real_t get_abs_pulse_time() const;

	// Sets the spin difference threshold, clamped to >= MIN_ABS_VALUE.
	// p_abs_spin_difference_threshold: spin difference magnitude in m/s that engages ABS.
	// Returns: nothing. Emits changed; MVWheel compares against it every physics tick.
	void set_abs_spin_difference_threshold(real_t p_abs_spin_difference_threshold);
	// Returns: the spin difference threshold in m/s.
	real_t get_abs_spin_difference_threshold() const;

	// Sets the bias weight, clamped to >= AUTOMATIC_BIAS_WEIGHT.
	// p_bias_weight: -1 for automatic, otherwise a relative weight >= 0.
	// Returns: nothing. Emits changed; MVVehicle renormalises the brake split on rebuild.
	void set_bias_weight(real_t p_bias_weight);
	// Returns: the bias weight, -1 for automatic.
	real_t get_bias_weight() const;
};

} // namespace godot
