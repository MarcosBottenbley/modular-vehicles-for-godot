// Terms: see mv_suspension_settings.h.

#include "mv_suspension_settings.h"

#include <godot_cpp/core/class_db.hpp>

#include "mv_settings_helpers.h"

using namespace godot;

void MVSuspensionSettings::set_spring_length(real_t p_spring_length) {
	if (mv_store_if_changed(spring_length, MAX(p_spring_length, MIN_SPRING_LENGTH))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_spring_length() const {
	return spring_length;
}

void MVSuspensionSettings::set_resting_ratio(real_t p_resting_ratio) {
	if (mv_store_if_changed(resting_ratio, CLAMP(p_resting_ratio, MIN_RESTING_RATIO, MAX_RESTING_RATIO))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_resting_ratio() const {
	return resting_ratio;
}

void MVSuspensionSettings::set_spring_rate_mode(SpringRateMode p_spring_rate_mode) {
	ERR_FAIL_INDEX(p_spring_rate_mode, SPRING_RATE_MODE_COUNT);
	if (!mv_store_if_changed(spring_rate_mode, p_spring_rate_mode)) {
		return;
	}
	// _validate_property depends on the mode, so the inspector must rebuild its list.
	notify_property_list_changed();
	emit_changed();
}

MVSuspensionSettings::SpringRateMode MVSuspensionSettings::get_spring_rate_mode() const {
	return spring_rate_mode;
}

void MVSuspensionSettings::set_manual_spring_rate(real_t p_manual_spring_rate) {
	if (mv_store_if_changed(manual_spring_rate, MAX(p_manual_spring_rate, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_manual_spring_rate() const {
	return manual_spring_rate;
}

void MVSuspensionSettings::set_damping_ratio(real_t p_damping_ratio) {
	if (mv_store_if_changed(damping_ratio, MAX(p_damping_ratio, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_damping_ratio() const {
	return damping_ratio;
}

void MVSuspensionSettings::set_bump_damp_multiplier(real_t p_bump_damp_multiplier) {
	if (mv_store_if_changed(bump_damp_multiplier, MAX(p_bump_damp_multiplier, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_bump_damp_multiplier() const {
	return bump_damp_multiplier;
}

void MVSuspensionSettings::set_rebound_damp_multiplier(real_t p_rebound_damp_multiplier) {
	if (mv_store_if_changed(rebound_damp_multiplier, MAX(p_rebound_damp_multiplier, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_rebound_damp_multiplier() const {
	return rebound_damp_multiplier;
}

void MVSuspensionSettings::set_fast_damp_threshold(real_t p_fast_damp_threshold) {
	if (mv_store_if_changed(fast_damp_threshold, MAX(p_fast_damp_threshold, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_fast_damp_threshold() const {
	return fast_damp_threshold;
}

void MVSuspensionSettings::set_manual_damping(real_t p_manual_damping) {
	if (mv_store_if_changed(manual_damping, MAX(p_manual_damping, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_manual_damping() const {
	return manual_damping;
}

void MVSuspensionSettings::set_bump_stop_multiplier(real_t p_bump_stop_multiplier) {
	if (mv_store_if_changed(bump_stop_multiplier, MAX(p_bump_stop_multiplier, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_bump_stop_multiplier() const {
	return bump_stop_multiplier;
}

void MVSuspensionSettings::set_anti_roll_ratio(real_t p_anti_roll_ratio) {
	if (mv_store_if_changed(anti_roll_ratio, MAX(p_anti_roll_ratio, MIN_RATE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_anti_roll_ratio() const {
	return anti_roll_ratio;
}

void MVSuspensionSettings::set_camber(real_t p_camber) {
	if (mv_store_if_changed(camber, CLAMP(p_camber, -MAX_ALIGNMENT_ANGLE, MAX_ALIGNMENT_ANGLE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_camber() const {
	return camber;
}

void MVSuspensionSettings::set_toe(real_t p_toe) {
	if (mv_store_if_changed(toe, CLAMP(p_toe, -MAX_ALIGNMENT_ANGLE, MAX_ALIGNMENT_ANGLE))) {
		emit_changed();
	}
}

real_t MVSuspensionSettings::get_toe() const {
	return toe;
}

void MVSuspensionSettings::_validate_property(PropertyInfo &p_property) const {
	const bool manual = spring_rate_mode == SPRING_RATE_MANUAL;
	const bool manual_input = p_property.name == StringName("manual_spring_rate") || p_property.name == StringName("manual_damping");
	const bool auto_input = p_property.name == StringName("resting_ratio") || p_property.name == StringName("damping_ratio");
	if ((manual_input && !manual) || (auto_input && manual)) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
}

void MVSuspensionSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_spring_length", "spring_length"), &MVSuspensionSettings::set_spring_length);
	ClassDB::bind_method(D_METHOD("get_spring_length"), &MVSuspensionSettings::get_spring_length);
	ClassDB::bind_method(D_METHOD("set_resting_ratio", "resting_ratio"), &MVSuspensionSettings::set_resting_ratio);
	ClassDB::bind_method(D_METHOD("get_resting_ratio"), &MVSuspensionSettings::get_resting_ratio);
	ClassDB::bind_method(D_METHOD("set_spring_rate_mode", "spring_rate_mode"), &MVSuspensionSettings::set_spring_rate_mode);
	ClassDB::bind_method(D_METHOD("get_spring_rate_mode"), &MVSuspensionSettings::get_spring_rate_mode);
	ClassDB::bind_method(D_METHOD("set_manual_spring_rate", "manual_spring_rate"), &MVSuspensionSettings::set_manual_spring_rate);
	ClassDB::bind_method(D_METHOD("get_manual_spring_rate"), &MVSuspensionSettings::get_manual_spring_rate);
	ClassDB::bind_method(D_METHOD("set_damping_ratio", "damping_ratio"), &MVSuspensionSettings::set_damping_ratio);
	ClassDB::bind_method(D_METHOD("get_damping_ratio"), &MVSuspensionSettings::get_damping_ratio);
	ClassDB::bind_method(D_METHOD("set_bump_damp_multiplier", "bump_damp_multiplier"), &MVSuspensionSettings::set_bump_damp_multiplier);
	ClassDB::bind_method(D_METHOD("get_bump_damp_multiplier"), &MVSuspensionSettings::get_bump_damp_multiplier);
	ClassDB::bind_method(D_METHOD("set_rebound_damp_multiplier", "rebound_damp_multiplier"), &MVSuspensionSettings::set_rebound_damp_multiplier);
	ClassDB::bind_method(D_METHOD("get_rebound_damp_multiplier"), &MVSuspensionSettings::get_rebound_damp_multiplier);
	ClassDB::bind_method(D_METHOD("set_fast_damp_threshold", "fast_damp_threshold"), &MVSuspensionSettings::set_fast_damp_threshold);
	ClassDB::bind_method(D_METHOD("get_fast_damp_threshold"), &MVSuspensionSettings::get_fast_damp_threshold);
	ClassDB::bind_method(D_METHOD("set_manual_damping", "manual_damping"), &MVSuspensionSettings::set_manual_damping);
	ClassDB::bind_method(D_METHOD("get_manual_damping"), &MVSuspensionSettings::get_manual_damping);
	ClassDB::bind_method(D_METHOD("set_bump_stop_multiplier", "bump_stop_multiplier"), &MVSuspensionSettings::set_bump_stop_multiplier);
	ClassDB::bind_method(D_METHOD("get_bump_stop_multiplier"), &MVSuspensionSettings::get_bump_stop_multiplier);
	ClassDB::bind_method(D_METHOD("set_anti_roll_ratio", "anti_roll_ratio"), &MVSuspensionSettings::set_anti_roll_ratio);
	ClassDB::bind_method(D_METHOD("get_anti_roll_ratio"), &MVSuspensionSettings::get_anti_roll_ratio);
	ClassDB::bind_method(D_METHOD("set_camber", "camber"), &MVSuspensionSettings::set_camber);
	ClassDB::bind_method(D_METHOD("get_camber"), &MVSuspensionSettings::get_camber);
	ClassDB::bind_method(D_METHOD("set_toe", "toe"), &MVSuspensionSettings::set_toe);
	ClassDB::bind_method(D_METHOD("get_toe"), &MVSuspensionSettings::get_toe);

	// Inspector maxima are slider ends only unless the setter clamps to the same value.
	// Angle hints are written in degrees: radians_as_degrees converts the stored radians.
	const double max_alignment_angle_degrees = Math::rad_to_deg((double)MAX_ALIGNMENT_ANGLE);
	ADD_GROUP("Spring", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_length", PROPERTY_HINT_RANGE, mv_range_hint(MIN_SPRING_LENGTH, 2.0, 0.001, "or_greater,suffix:m")), "set_spring_length", "get_spring_length");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "spring_rate_mode", PROPERTY_HINT_ENUM, "Auto From Load,Manual"), "set_spring_rate_mode", "get_spring_rate_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "resting_ratio", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RESTING_RATIO, MAX_RESTING_RATIO, 0.01)), "set_resting_ratio", "get_resting_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "manual_spring_rate", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 200000.0, 1.0, "or_greater,suffix:N/m")), "set_manual_spring_rate", "get_manual_spring_rate");
	ADD_GROUP("Damping", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping_ratio", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 2.0, 0.01, "or_greater")), "set_damping_ratio", "get_damping_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "manual_damping", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 20000.0, 1.0, "or_greater,suffix:N⋅s/m")), "set_manual_damping", "get_manual_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bump_damp_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 5.0, 0.0001, "or_greater")), "set_bump_damp_multiplier", "get_bump_damp_multiplier");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rebound_damp_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 5.0, 0.0001, "or_greater")), "set_rebound_damp_multiplier", "get_rebound_damp_multiplier");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fast_damp_threshold", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 2.0, 0.001, "or_greater,suffix:m/s")), "set_fast_damp_threshold", "get_fast_damp_threshold");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bump_stop_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 5.0, 0.01, "or_greater")), "set_bump_stop_multiplier", "get_bump_stop_multiplier");
	ADD_GROUP("Geometry", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "anti_roll_ratio", PROPERTY_HINT_RANGE, mv_range_hint(MIN_RATE, 2.0, 0.01, "or_greater")), "set_anti_roll_ratio", "get_anti_roll_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "camber", PROPERTY_HINT_RANGE, mv_range_hint(-max_alignment_angle_degrees, max_alignment_angle_degrees, 0.01, "radians_as_degrees")), "set_camber", "get_camber");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "toe", PROPERTY_HINT_RANGE, mv_range_hint(-max_alignment_angle_degrees, max_alignment_angle_degrees, 0.01, "radians_as_degrees")), "set_toe", "get_toe");

	BIND_ENUM_CONSTANT(SPRING_RATE_AUTO_FROM_LOAD);
	BIND_ENUM_CONSTANT(SPRING_RATE_MANUAL);
}
