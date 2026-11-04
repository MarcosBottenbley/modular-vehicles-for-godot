#include "mv_suspension_settings.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>

using namespace godot;

void MVSuspensionSettings::set_spring_length(real_t p_spring_length) {
	p_spring_length = MAX(p_spring_length, (real_t)0.001);
	if (spring_length == p_spring_length) {
		return;
	}
	spring_length = p_spring_length;
	emit_changed();
}

real_t MVSuspensionSettings::get_spring_length() const {
	return spring_length;
}

void MVSuspensionSettings::set_resting_ratio(real_t p_resting_ratio) {
	p_resting_ratio = CLAMP(p_resting_ratio, (real_t)0.01, (real_t)1.0);
	if (resting_ratio == p_resting_ratio) {
		return;
	}
	resting_ratio = p_resting_ratio;
	emit_changed();
}

real_t MVSuspensionSettings::get_resting_ratio() const {
	return resting_ratio;
}

void MVSuspensionSettings::set_spring_rate_mode(SpringRateMode p_spring_rate_mode) {
	ERR_FAIL_INDEX(p_spring_rate_mode, 2);
	if (spring_rate_mode == p_spring_rate_mode) {
		return;
	}
	spring_rate_mode = p_spring_rate_mode;
	notify_property_list_changed();
	emit_changed();
}

MVSuspensionSettings::SpringRateMode MVSuspensionSettings::get_spring_rate_mode() const {
	return spring_rate_mode;
}

void MVSuspensionSettings::set_manual_spring_rate(real_t p_manual_spring_rate) {
	p_manual_spring_rate = MAX(p_manual_spring_rate, (real_t)0.0);
	if (manual_spring_rate == p_manual_spring_rate) {
		return;
	}
	manual_spring_rate = p_manual_spring_rate;
	emit_changed();
}

real_t MVSuspensionSettings::get_manual_spring_rate() const {
	return manual_spring_rate;
}

void MVSuspensionSettings::set_damping_ratio(real_t p_damping_ratio) {
	p_damping_ratio = MAX(p_damping_ratio, (real_t)0.0);
	if (damping_ratio == p_damping_ratio) {
		return;
	}
	damping_ratio = p_damping_ratio;
	emit_changed();
}

real_t MVSuspensionSettings::get_damping_ratio() const {
	return damping_ratio;
}

void MVSuspensionSettings::set_bump_damp_multiplier(real_t p_bump_damp_multiplier) {
	p_bump_damp_multiplier = MAX(p_bump_damp_multiplier, (real_t)0.0);
	if (bump_damp_multiplier == p_bump_damp_multiplier) {
		return;
	}
	bump_damp_multiplier = p_bump_damp_multiplier;
	emit_changed();
}

real_t MVSuspensionSettings::get_bump_damp_multiplier() const {
	return bump_damp_multiplier;
}

void MVSuspensionSettings::set_rebound_damp_multiplier(real_t p_rebound_damp_multiplier) {
	p_rebound_damp_multiplier = MAX(p_rebound_damp_multiplier, (real_t)0.0);
	if (rebound_damp_multiplier == p_rebound_damp_multiplier) {
		return;
	}
	rebound_damp_multiplier = p_rebound_damp_multiplier;
	emit_changed();
}

real_t MVSuspensionSettings::get_rebound_damp_multiplier() const {
	return rebound_damp_multiplier;
}

void MVSuspensionSettings::set_fast_damp_threshold(real_t p_fast_damp_threshold) {
	p_fast_damp_threshold = MAX(p_fast_damp_threshold, (real_t)0.0);
	if (fast_damp_threshold == p_fast_damp_threshold) {
		return;
	}
	fast_damp_threshold = p_fast_damp_threshold;
	emit_changed();
}

real_t MVSuspensionSettings::get_fast_damp_threshold() const {
	return fast_damp_threshold;
}

void MVSuspensionSettings::set_manual_damping(real_t p_manual_damping) {
	p_manual_damping = MAX(p_manual_damping, (real_t)0.0);
	if (manual_damping == p_manual_damping) {
		return;
	}
	manual_damping = p_manual_damping;
	emit_changed();
}

real_t MVSuspensionSettings::get_manual_damping() const {
	return manual_damping;
}

void MVSuspensionSettings::set_bump_stop_multiplier(real_t p_bump_stop_multiplier) {
	p_bump_stop_multiplier = MAX(p_bump_stop_multiplier, (real_t)0.0);
	if (bump_stop_multiplier == p_bump_stop_multiplier) {
		return;
	}
	bump_stop_multiplier = p_bump_stop_multiplier;
	emit_changed();
}

real_t MVSuspensionSettings::get_bump_stop_multiplier() const {
	return bump_stop_multiplier;
}

void MVSuspensionSettings::set_anti_roll_ratio(real_t p_anti_roll_ratio) {
	p_anti_roll_ratio = MAX(p_anti_roll_ratio, (real_t)0.0);
	if (anti_roll_ratio == p_anti_roll_ratio) {
		return;
	}
	anti_roll_ratio = p_anti_roll_ratio;
	emit_changed();
}

real_t MVSuspensionSettings::get_anti_roll_ratio() const {
	return anti_roll_ratio;
}

void MVSuspensionSettings::set_camber(real_t p_camber) {
	p_camber = CLAMP(p_camber, (real_t)-Math_PI * 0.25, (real_t)Math_PI * 0.25);
	if (camber == p_camber) {
		return;
	}
	camber = p_camber;
	emit_changed();
}

real_t MVSuspensionSettings::get_camber() const {
	return camber;
}

void MVSuspensionSettings::set_toe(real_t p_toe) {
	p_toe = CLAMP(p_toe, (real_t)-Math_PI * 0.25, (real_t)Math_PI * 0.25);
	if (toe == p_toe) {
		return;
	}
	toe = p_toe;
	emit_changed();
}

real_t MVSuspensionSettings::get_toe() const {
	return toe;
}

void MVSuspensionSettings::_validate_property(PropertyInfo &p_property) const {
	// Only the inputs of the active spring rate mode are shown in the inspector.
	const bool manual = spring_rate_mode == SPRING_RATE_MANUAL;
	if (p_property.name == StringName("manual_spring_rate") || p_property.name == StringName("manual_damping")) {
		if (!manual) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
	} else if (p_property.name == StringName("resting_ratio") || p_property.name == StringName("damping_ratio")) {
		if (manual) {
			p_property.usage = PROPERTY_USAGE_NO_EDITOR;
		}
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

	ADD_GROUP("Spring", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_length", PROPERTY_HINT_RANGE, "0.001,2,0.001,or_greater,suffix:m"), "set_spring_length", "get_spring_length");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "spring_rate_mode", PROPERTY_HINT_ENUM, "Auto From Load,Manual"), "set_spring_rate_mode", "get_spring_rate_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "resting_ratio", PROPERTY_HINT_RANGE, "0.01,1,0.01"), "set_resting_ratio", "get_resting_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "manual_spring_rate", PROPERTY_HINT_RANGE, "0,200000,1,or_greater,suffix:N/m"), "set_manual_spring_rate", "get_manual_spring_rate");
	ADD_GROUP("Damping", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping_ratio", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_damping_ratio", "get_damping_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "manual_damping", PROPERTY_HINT_RANGE, "0,20000,1,or_greater,suffix:N⋅s/m"), "set_manual_damping", "get_manual_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bump_damp_multiplier", PROPERTY_HINT_RANGE, "0,5,0.0001,or_greater"), "set_bump_damp_multiplier", "get_bump_damp_multiplier");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rebound_damp_multiplier", PROPERTY_HINT_RANGE, "0,5,0.0001,or_greater"), "set_rebound_damp_multiplier", "get_rebound_damp_multiplier");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fast_damp_threshold", PROPERTY_HINT_RANGE, "0,2,0.001,or_greater,suffix:m/s"), "set_fast_damp_threshold", "get_fast_damp_threshold");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bump_stop_multiplier", PROPERTY_HINT_RANGE, "0,5,0.01,or_greater"), "set_bump_stop_multiplier", "get_bump_stop_multiplier");
	ADD_GROUP("Geometry", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "anti_roll_ratio", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_anti_roll_ratio", "get_anti_roll_ratio");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "camber", PROPERTY_HINT_RANGE, "-45,45,0.01,radians_as_degrees"), "set_camber", "get_camber");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "toe", PROPERTY_HINT_RANGE, "-45,45,0.01,radians_as_degrees"), "set_toe", "get_toe");

	BIND_ENUM_CONSTANT(SPRING_RATE_AUTO_FROM_LOAD);
	BIND_ENUM_CONSTANT(SPRING_RATE_MANUAL);
}
