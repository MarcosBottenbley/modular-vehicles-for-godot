#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/variant/string_name.hpp>

namespace godot {

// Friction and stiffness data for one driving surface. A wheel picks the
// surface whose name matches a node group of the body it is touching.
class MVSurface : public Resource {
	GDCLASS(MVSurface, Resource)

	StringName name;
	real_t tire_stiffness = 10.0;
	real_t coefficient_of_friction = 3.0;
	real_t rolling_resistance = 1.0;
	real_t lateral_grip_assist = 0.05;
	real_t longitudinal_grip_ratio = 0.5;

protected:
	static void _bind_methods();

public:
	MVSurface();

	void set_name(const StringName &p_name);
	StringName get_name() const;

	void set_tire_stiffness(real_t p_tire_stiffness);
	real_t get_tire_stiffness() const;

	void set_coefficient_of_friction(real_t p_coefficient_of_friction);
	real_t get_coefficient_of_friction() const;

	void set_rolling_resistance(real_t p_rolling_resistance);
	real_t get_rolling_resistance() const;

	void set_lateral_grip_assist(real_t p_lateral_grip_assist);
	real_t get_lateral_grip_assist() const;

	void set_longitudinal_grip_ratio(real_t p_longitudinal_grip_ratio);
	real_t get_longitudinal_grip_ratio() const;
};

} // namespace godot
