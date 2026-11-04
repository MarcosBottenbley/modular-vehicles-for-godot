#pragma once

#include <godot_cpp/classes/resource.hpp>

namespace godot {

// Per-wheel brake behaviour: anti-lock braking and the share of the total
// brake force this wheel receives.
class MVBrakeSettings : public Resource {
	GDCLASS(MVBrakeSettings, Resource)

	bool abs_enabled = true;
	real_t abs_pulse_time = 0.03;
	real_t abs_spin_difference_threshold = 12.0;
	real_t bias_weight = -1.0;

protected:
	static void _bind_methods();

public:
	void set_abs_enabled(bool p_abs_enabled);
	bool is_abs_enabled() const;

	void set_abs_pulse_time(real_t p_abs_pulse_time);
	real_t get_abs_pulse_time() const;

	void set_abs_spin_difference_threshold(real_t p_abs_spin_difference_threshold);
	real_t get_abs_spin_difference_threshold() const;

	void set_bias_weight(real_t p_bias_weight);
	real_t get_bias_weight() const;
};

} // namespace godot
