#include "mv_vehicle.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void MVVehicle::set_center_of_gravity(const Vector3 &p_center_of_gravity) {
	center_of_gravity = p_center_of_gravity;
	set_center_of_mass_mode(RigidBody3D::CENTER_OF_MASS_MODE_CUSTOM);
	set_center_of_mass(center_of_gravity);
}

Vector3 MVVehicle::get_center_of_gravity() const {
	return center_of_gravity;
}

void MVVehicle::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_center_of_gravity", "center_of_gravity"), &MVVehicle::set_center_of_gravity);
	ClassDB::bind_method(D_METHOD("get_center_of_gravity"), &MVVehicle::get_center_of_gravity);

	ADD_GROUP("Body", "");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "center_of_gravity", PROPERTY_HINT_NONE, "suffix:m"), "set_center_of_gravity", "get_center_of_gravity");
}
