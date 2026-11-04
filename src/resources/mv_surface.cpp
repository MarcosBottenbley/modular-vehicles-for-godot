#include "mv_surface.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

MVSurface::MVSurface() :
		name("Road") {
}

void MVSurface::set_name(const StringName &p_name) {
	if (name == p_name) {
		return;
	}
	name = p_name;
	emit_changed();
}

StringName MVSurface::get_name() const {
	return name;
}

void MVSurface::set_tire_stiffness(real_t p_tire_stiffness) {
	p_tire_stiffness = MAX(p_tire_stiffness, (real_t)0.0);
	if (tire_stiffness == p_tire_stiffness) {
		return;
	}
	tire_stiffness = p_tire_stiffness;
	emit_changed();
}

real_t MVSurface::get_tire_stiffness() const {
	return tire_stiffness;
}

void MVSurface::set_coefficient_of_friction(real_t p_coefficient_of_friction) {
	p_coefficient_of_friction = MAX(p_coefficient_of_friction, (real_t)0.0);
	if (coefficient_of_friction == p_coefficient_of_friction) {
		return;
	}
	coefficient_of_friction = p_coefficient_of_friction;
	emit_changed();
}

real_t MVSurface::get_coefficient_of_friction() const {
	return coefficient_of_friction;
}

void MVSurface::set_rolling_resistance(real_t p_rolling_resistance) {
	p_rolling_resistance = MAX(p_rolling_resistance, (real_t)0.0);
	if (rolling_resistance == p_rolling_resistance) {
		return;
	}
	rolling_resistance = p_rolling_resistance;
	emit_changed();
}

real_t MVSurface::get_rolling_resistance() const {
	return rolling_resistance;
}

void MVSurface::set_lateral_grip_assist(real_t p_lateral_grip_assist) {
	p_lateral_grip_assist = MAX(p_lateral_grip_assist, (real_t)0.0);
	if (lateral_grip_assist == p_lateral_grip_assist) {
		return;
	}
	lateral_grip_assist = p_lateral_grip_assist;
	emit_changed();
}

real_t MVSurface::get_lateral_grip_assist() const {
	return lateral_grip_assist;
}

void MVSurface::set_longitudinal_grip_ratio(real_t p_longitudinal_grip_ratio) {
	p_longitudinal_grip_ratio = MAX(p_longitudinal_grip_ratio, (real_t)0.0);
	if (longitudinal_grip_ratio == p_longitudinal_grip_ratio) {
		return;
	}
	longitudinal_grip_ratio = p_longitudinal_grip_ratio;
	emit_changed();
}

real_t MVSurface::get_longitudinal_grip_ratio() const {
	return longitudinal_grip_ratio;
}

void MVSurface::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_name", "name"), &MVSurface::set_name);
	ClassDB::bind_method(D_METHOD("get_name"), &MVSurface::get_name);
	ClassDB::bind_method(D_METHOD("set_tire_stiffness", "tire_stiffness"), &MVSurface::set_tire_stiffness);
	ClassDB::bind_method(D_METHOD("get_tire_stiffness"), &MVSurface::get_tire_stiffness);
	ClassDB::bind_method(D_METHOD("set_coefficient_of_friction", "coefficient_of_friction"), &MVSurface::set_coefficient_of_friction);
	ClassDB::bind_method(D_METHOD("get_coefficient_of_friction"), &MVSurface::get_coefficient_of_friction);
	ClassDB::bind_method(D_METHOD("set_rolling_resistance", "rolling_resistance"), &MVSurface::set_rolling_resistance);
	ClassDB::bind_method(D_METHOD("get_rolling_resistance"), &MVSurface::get_rolling_resistance);
	ClassDB::bind_method(D_METHOD("set_lateral_grip_assist", "lateral_grip_assist"), &MVSurface::set_lateral_grip_assist);
	ClassDB::bind_method(D_METHOD("get_lateral_grip_assist"), &MVSurface::get_lateral_grip_assist);
	ClassDB::bind_method(D_METHOD("set_longitudinal_grip_ratio", "longitudinal_grip_ratio"), &MVSurface::set_longitudinal_grip_ratio);
	ClassDB::bind_method(D_METHOD("get_longitudinal_grip_ratio"), &MVSurface::get_longitudinal_grip_ratio);

	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "name"), "set_name", "get_name");
	ADD_GROUP("Grip", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_stiffness", PROPERTY_HINT_RANGE, "0,100,0.01,or_greater"), "set_tire_stiffness", "get_tire_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "coefficient_of_friction", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_coefficient_of_friction", "get_coefficient_of_friction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rolling_resistance", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_rolling_resistance", "get_rolling_resistance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lateral_grip_assist", PROPERTY_HINT_RANGE, "0,1,0.001,or_greater"), "set_lateral_grip_assist", "get_lateral_grip_assist");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "longitudinal_grip_ratio", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_longitudinal_grip_ratio", "get_longitudinal_grip_ratio");
}
