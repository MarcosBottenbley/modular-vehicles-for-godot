#pragma once

#include <godot_cpp/classes/rigid_body3d.hpp>

namespace godot {

// Root node of a modular vehicle. It will discover MVWheel descendants and run
// the vehicle simulation. This scaffold version only exposes the center of
// gravity so the class appears in the editor; the physics arrives in later
// branches (see docs/ and the Trello board).
class MVVehicle : public RigidBody3D {
	GDCLASS(MVVehicle, RigidBody3D)

	Vector3 center_of_gravity;

protected:
	static void _bind_methods();

public:
	void set_center_of_gravity(const Vector3 &p_center_of_gravity);
	Vector3 get_center_of_gravity() const;
};

} // namespace godot
