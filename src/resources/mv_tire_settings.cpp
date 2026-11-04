// Terms: see mv_tire_settings.h.

#include "mv_tire_settings.h"

#include <godot_cpp/core/class_db.hpp>

#include "mv_settings_helpers.h"

using namespace godot;

void MVTireSettings::set_tire_radius(real_t p_tire_radius) {
	if (mv_store_if_changed(tire_radius, MAX(p_tire_radius, MIN_TIRE_RADIUS))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_tire_radius() const {
	return tire_radius;
}

void MVTireSettings::set_tire_width(real_t p_tire_width) {
	if (mv_store_if_changed(tire_width, MAX(p_tire_width, MIN_TIRE_WIDTH))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_tire_width() const {
	return tire_width;
}

void MVTireSettings::set_wheel_mass(real_t p_wheel_mass) {
	if (mv_store_if_changed(wheel_mass, MAX(p_wheel_mass, MIN_WHEEL_MASS))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_wheel_mass() const {
	return wheel_mass;
}

void MVTireSettings::set_contact_patch(real_t p_contact_patch) {
	if (mv_store_if_changed(contact_patch, MAX(p_contact_patch, MIN_CONTACT_PATCH))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_contact_patch() const {
	return contact_patch;
}

void MVTireSettings::set_braking_grip_multiplier(real_t p_braking_grip_multiplier) {
	if (mv_store_if_changed(braking_grip_multiplier, MAX(p_braking_grip_multiplier, MIN_MULTIPLIER))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_braking_grip_multiplier() const {
	return braking_grip_multiplier;
}

void MVTireSettings::set_wheel_to_body_torque_multiplier(real_t p_wheel_to_body_torque_multiplier) {
	if (mv_store_if_changed(wheel_to_body_torque_multiplier, MAX(p_wheel_to_body_torque_multiplier, MIN_MULTIPLIER))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_wheel_to_body_torque_multiplier() const {
	return wheel_to_body_torque_multiplier;
}

void MVTireSettings::set_grip_multiplier(real_t p_grip_multiplier) {
	if (mv_store_if_changed(grip_multiplier, MAX(p_grip_multiplier, MIN_MULTIPLIER))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_grip_multiplier() const {
	return grip_multiplier;
}

void MVTireSettings::set_stiffness_multiplier(real_t p_stiffness_multiplier) {
	if (mv_store_if_changed(stiffness_multiplier, MAX(p_stiffness_multiplier, MIN_MULTIPLIER))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_stiffness_multiplier() const {
	return stiffness_multiplier;
}

void MVTireSettings::set_rolling_resistance_multiplier(real_t p_rolling_resistance_multiplier) {
	if (mv_store_if_changed(rolling_resistance_multiplier, MAX(p_rolling_resistance_multiplier, MIN_MULTIPLIER))) {
		emit_changed();
	}
}

real_t MVTireSettings::get_rolling_resistance_multiplier() const {
	return rolling_resistance_multiplier;
}

void MVTireSettings::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_tire_radius", "tire_radius"), &MVTireSettings::set_tire_radius);
	ClassDB::bind_method(D_METHOD("get_tire_radius"), &MVTireSettings::get_tire_radius);
	ClassDB::bind_method(D_METHOD("set_tire_width", "tire_width"), &MVTireSettings::set_tire_width);
	ClassDB::bind_method(D_METHOD("get_tire_width"), &MVTireSettings::get_tire_width);
	ClassDB::bind_method(D_METHOD("set_wheel_mass", "wheel_mass"), &MVTireSettings::set_wheel_mass);
	ClassDB::bind_method(D_METHOD("get_wheel_mass"), &MVTireSettings::get_wheel_mass);
	ClassDB::bind_method(D_METHOD("set_contact_patch", "contact_patch"), &MVTireSettings::set_contact_patch);
	ClassDB::bind_method(D_METHOD("get_contact_patch"), &MVTireSettings::get_contact_patch);
	ClassDB::bind_method(D_METHOD("set_braking_grip_multiplier", "braking_grip_multiplier"), &MVTireSettings::set_braking_grip_multiplier);
	ClassDB::bind_method(D_METHOD("get_braking_grip_multiplier"), &MVTireSettings::get_braking_grip_multiplier);
	ClassDB::bind_method(D_METHOD("set_wheel_to_body_torque_multiplier", "wheel_to_body_torque_multiplier"), &MVTireSettings::set_wheel_to_body_torque_multiplier);
	ClassDB::bind_method(D_METHOD("get_wheel_to_body_torque_multiplier"), &MVTireSettings::get_wheel_to_body_torque_multiplier);
	ClassDB::bind_method(D_METHOD("set_grip_multiplier", "grip_multiplier"), &MVTireSettings::set_grip_multiplier);
	ClassDB::bind_method(D_METHOD("get_grip_multiplier"), &MVTireSettings::get_grip_multiplier);
	ClassDB::bind_method(D_METHOD("set_stiffness_multiplier", "stiffness_multiplier"), &MVTireSettings::set_stiffness_multiplier);
	ClassDB::bind_method(D_METHOD("get_stiffness_multiplier"), &MVTireSettings::get_stiffness_multiplier);
	ClassDB::bind_method(D_METHOD("set_rolling_resistance_multiplier", "rolling_resistance_multiplier"), &MVTireSettings::set_rolling_resistance_multiplier);
	ClassDB::bind_method(D_METHOD("get_rolling_resistance_multiplier"), &MVTireSettings::get_rolling_resistance_multiplier);

	// Inspector maxima are slider ends only; every value accepts larger numbers.
	ADD_GROUP("Geometry", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_radius", PROPERTY_HINT_RANGE, mv_range_hint(MIN_TIRE_RADIUS, 2.0, 0.001, "or_greater,suffix:m")), "set_tire_radius", "get_tire_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tire_width", PROPERTY_HINT_RANGE, mv_range_hint(MIN_TIRE_WIDTH, 1000.0, 1.0, "or_greater,suffix:mm")), "set_tire_width", "get_tire_width");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_mass", PROPERTY_HINT_RANGE, mv_range_hint(MIN_WHEEL_MASS, 500.0, 0.1, "or_greater,suffix:kg")), "set_wheel_mass", "get_wheel_mass");
	ADD_GROUP("Brush Model", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "contact_patch", PROPERTY_HINT_RANGE, mv_range_hint(MIN_CONTACT_PATCH, 1.0, 0.001, "or_greater,suffix:m")), "set_contact_patch", "get_contact_patch");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "braking_grip_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_MULTIPLIER, 5.0, 0.01, "or_greater")), "set_braking_grip_multiplier", "get_braking_grip_multiplier");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wheel_to_body_torque_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_MULTIPLIER, 2.0, 0.01, "or_greater")), "set_wheel_to_body_torque_multiplier", "get_wheel_to_body_torque_multiplier");
	ADD_GROUP("Surface Multipliers", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "grip_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_MULTIPLIER, 5.0, 0.01, "or_greater")), "set_grip_multiplier", "get_grip_multiplier");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_MULTIPLIER, 5.0, 0.01, "or_greater")), "set_stiffness_multiplier", "get_stiffness_multiplier");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rolling_resistance_multiplier", PROPERTY_HINT_RANGE, mv_range_hint(MIN_MULTIPLIER, 5.0, 0.01, "or_greater")), "set_rolling_resistance_multiplier", "get_rolling_resistance_multiplier");
}
