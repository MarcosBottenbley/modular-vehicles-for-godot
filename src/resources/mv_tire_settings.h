#pragma once

#include <godot_cpp/classes/resource.hpp>

namespace godot {

// Tire geometry and brush-model parameters shared by any number of wheels.
// The surface multipliers scale the matching MVSurface values for this tire.
class MVTireSettings : public Resource {
	GDCLASS(MVTireSettings, Resource)

	real_t tire_radius = 0.3;
	real_t tire_width = 245.0;
	real_t wheel_mass = 15.0;
	real_t contact_patch = 0.2;
	real_t braking_grip_multiplier = 1.5;
	real_t wheel_to_body_torque_multiplier = 1.0;
	real_t grip_multiplier = 1.0;
	real_t stiffness_multiplier = 1.0;
	real_t rolling_resistance_multiplier = 1.0;

protected:
	static void _bind_methods();

public:
	void set_tire_radius(real_t p_tire_radius);
	real_t get_tire_radius() const;

	void set_tire_width(real_t p_tire_width);
	real_t get_tire_width() const;

	void set_wheel_mass(real_t p_wheel_mass);
	real_t get_wheel_mass() const;

	void set_contact_patch(real_t p_contact_patch);
	real_t get_contact_patch() const;

	void set_braking_grip_multiplier(real_t p_braking_grip_multiplier);
	real_t get_braking_grip_multiplier() const;

	void set_wheel_to_body_torque_multiplier(real_t p_wheel_to_body_torque_multiplier);
	real_t get_wheel_to_body_torque_multiplier() const;

	void set_grip_multiplier(real_t p_grip_multiplier);
	real_t get_grip_multiplier() const;

	void set_stiffness_multiplier(real_t p_stiffness_multiplier);
	real_t get_stiffness_multiplier() const;

	void set_rolling_resistance_multiplier(real_t p_rolling_resistance_multiplier);
	real_t get_rolling_resistance_multiplier() const;
};

} // namespace godot
