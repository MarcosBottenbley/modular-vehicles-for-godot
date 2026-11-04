// Terms: see mv_brake_settings.h.

#include "mv_brake_settings.h"

#include <godot_cpp/core/class_db.hpp>

#include "mv_settings_helpers.h"

using namespace godot;

void MVBrakeSettings::set_abs_enabled(bool p_abs_enabled) {
	if (mv_store_if_changed(abs_enabled, p_abs_enabled)) {
		emit_changed();
	}
}

bool MVBrakeSettings::is_abs_enabled() const {
	return abs_enabled;
}

void MVBrakeSettings::set_abs_pulse_time(real_t p_abs_pulse_time) {
	if (mv_store_if_changed(abs_pulse_time, MAX(p_abs_pulse_time, MIN_ABS_VALUE))) {
		emit_changed();
	}
}

real_t MVBrakeSettings::get_abs_pulse_time() const {
	return abs_pulse_time;
}

void MVBrakeSettings::set_abs_spin_difference_threshold(real_t p_abs_spin_difference_threshold) {
	if (mv_store_if_changed(abs_spin_difference_threshold, MAX(p_abs_spin_difference_threshold, MIN_ABS_VALUE))) {
		emit_changed();
	}
}

real_t MVBrakeSettings::get_abs_spin_difference_threshold() const {
	return abs_spin_difference_threshold;
}

void MVBrakeSettings::set_bias_weight(real_t p_bias_weight) {
	// Anything below the sentinel has no meaning, so it collapses to "automatic".
	if (mv_store_if_changed(bias_weight, MAX(p_bias_weight, AUTOMATIC_BIAS_WEIGHT))) {
		emit_changed();
	}
}

real_t MVBrakeSettings::get_bias_weight() const {
	return bias_weight;
}

void MVBrakeSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_abs_enabled", "abs_enabled"), &MVBrakeSettings::set_abs_enabled);
	ClassDB::bind_method(D_METHOD("is_abs_enabled"), &MVBrakeSettings::is_abs_enabled);
	ClassDB::bind_method(D_METHOD("set_abs_pulse_time", "abs_pulse_time"), &MVBrakeSettings::set_abs_pulse_time);
	ClassDB::bind_method(D_METHOD("get_abs_pulse_time"), &MVBrakeSettings::get_abs_pulse_time);
	ClassDB::bind_method(D_METHOD("set_abs_spin_difference_threshold", "abs_spin_difference_threshold"), &MVBrakeSettings::set_abs_spin_difference_threshold);
	ClassDB::bind_method(D_METHOD("get_abs_spin_difference_threshold"), &MVBrakeSettings::get_abs_spin_difference_threshold);
	ClassDB::bind_method(D_METHOD("set_bias_weight", "bias_weight"), &MVBrakeSettings::set_bias_weight);
	ClassDB::bind_method(D_METHOD("get_bias_weight"), &MVBrakeSettings::get_bias_weight);

	// Inspector maxima are slider ends only; every value accepts larger numbers.
	ADD_GROUP("ABS", "abs_");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "abs_enabled"), "set_abs_enabled", "is_abs_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "abs_pulse_time", PROPERTY_HINT_RANGE, mv_range_hint(MIN_ABS_VALUE, 1.0, 0.001, "or_greater,suffix:s")), "set_abs_pulse_time", "get_abs_pulse_time");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "abs_spin_difference_threshold", PROPERTY_HINT_RANGE, mv_range_hint(MIN_ABS_VALUE, 50.0, 0.1, "or_greater,suffix:m/s")), "set_abs_spin_difference_threshold", "get_abs_spin_difference_threshold");
	ADD_GROUP("Bias", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bias_weight", PROPERTY_HINT_RANGE, mv_range_hint(AUTOMATIC_BIAS_WEIGHT, 10.0, 0.01, "or_greater")), "set_bias_weight", "get_bias_weight");
}
