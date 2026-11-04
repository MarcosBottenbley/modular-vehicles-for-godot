#pragma once

#include <godot_cpp/classes/resource.hpp>

namespace godot {

// Spring, damper, anti-roll and alignment parameters for a wheel. Angles are
// stored in radians as magnitudes; the wheel applies the sign for its side.
class MVSuspensionSettings : public Resource {
	GDCLASS(MVSuspensionSettings, Resource)

public:
	enum SpringRateMode {
		SPRING_RATE_AUTO_FROM_LOAD,
		SPRING_RATE_MANUAL,
	};

private:
	real_t spring_length = 0.15;
	real_t resting_ratio = 0.5;
	SpringRateMode spring_rate_mode = SPRING_RATE_AUTO_FROM_LOAD;
	real_t manual_spring_rate = 49000.0;
	real_t damping_ratio = 0.4;
	real_t bump_damp_multiplier = 0.6667;
	real_t rebound_damp_multiplier = 1.5;
	real_t fast_damp_threshold = 0.127;
	real_t manual_damping = 3400.0;
	real_t bump_stop_multiplier = 1.0;
	real_t anti_roll_ratio = 0.25;
	real_t camber = 0.01745329;
	real_t toe = 0.01;

protected:
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

public:
	void set_spring_length(real_t p_spring_length);
	real_t get_spring_length() const;

	void set_resting_ratio(real_t p_resting_ratio);
	real_t get_resting_ratio() const;

	void set_spring_rate_mode(SpringRateMode p_spring_rate_mode);
	SpringRateMode get_spring_rate_mode() const;

	void set_manual_spring_rate(real_t p_manual_spring_rate);
	real_t get_manual_spring_rate() const;

	void set_damping_ratio(real_t p_damping_ratio);
	real_t get_damping_ratio() const;

	void set_bump_damp_multiplier(real_t p_bump_damp_multiplier);
	real_t get_bump_damp_multiplier() const;

	void set_rebound_damp_multiplier(real_t p_rebound_damp_multiplier);
	real_t get_rebound_damp_multiplier() const;

	void set_fast_damp_threshold(real_t p_fast_damp_threshold);
	real_t get_fast_damp_threshold() const;

	void set_manual_damping(real_t p_manual_damping);
	real_t get_manual_damping() const;

	void set_bump_stop_multiplier(real_t p_bump_stop_multiplier);
	real_t get_bump_stop_multiplier() const;

	void set_anti_roll_ratio(real_t p_anti_roll_ratio);
	real_t get_anti_roll_ratio() const;

	void set_camber(real_t p_camber);
	real_t get_camber() const;

	void set_toe(real_t p_toe);
	real_t get_toe() const;
};

} // namespace godot

VARIANT_ENUM_CAST(MVSuspensionSettings::SpringRateMode);
