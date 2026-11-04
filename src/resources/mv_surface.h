#pragma once

// Terms:
//   surface: one kind of ground a tire can roll on (road, dirt, grass ...), described by the
//     grip numbers below. Unitless multipliers unless stated.
//   node group: a Godot group name on a collider body. A wheel takes the first group of the body
//     it touches and looks a surface up by that name.
//   tire stiffness: the brush tire model's stiffness factor. The wheel scales it to N/m as
//     1e6 + 8e6 * tire_stiffness, matching wheel.gd.
//   coefficient of friction: multiplier on the force a loaded tire can apply on this surface.
//   rolling resistance: multiplier on the speed-dependent rolling resistance force.
//   lateral grip assist: extra lateral grip proportional to the lateral slip angle, in 1/rad.
//   longitudinal grip ratio: longitudinal grip as a fraction of lateral grip.

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/variant/string_name.hpp>

namespace godot {

// Grip and rolling resistance of one surface. MVSurfaceLibrary collects surfaces and MVWheel
// reads the matched surface's values every physics tick, scaled by its MVTireSettings
// multipliers. A default-constructed MVSurface is the GDScript "Road" entry.
class MVSurface : public Resource {
	GDCLASS(MVSurface, Resource)

public:
	static constexpr const char *DEFAULT_NAME = "Road"; // node group name
	static constexpr real_t DEFAULT_TIRE_STIFFNESS = 10.0; // unitless
	static constexpr real_t DEFAULT_COEFFICIENT_OF_FRICTION = 3.0; // unitless
	static constexpr real_t DEFAULT_ROLLING_RESISTANCE = 1.0; // unitless
	static constexpr real_t DEFAULT_LATERAL_GRIP_ASSIST = 0.05; // 1/rad
	static constexpr real_t DEFAULT_LONGITUDINAL_GRIP_RATIO = 0.5; // unitless
	// Every grip value is a multiplier; a negative one would flip forces, so 0 is the floor.
	static constexpr real_t MIN_GRIP_VALUE = 0.0;

private:
	StringName name;
	real_t tire_stiffness = DEFAULT_TIRE_STIFFNESS;
	real_t coefficient_of_friction = DEFAULT_COEFFICIENT_OF_FRICTION;
	real_t rolling_resistance = DEFAULT_ROLLING_RESISTANCE;
	real_t lateral_grip_assist = DEFAULT_LATERAL_GRIP_ASSIST;
	real_t longitudinal_grip_ratio = DEFAULT_LONGITUDINAL_GRIP_RATIO;

protected:
	static void _bind_methods();

public:
	// Starts the surface as the GDScript "Road" entry so an unnamed surface still matches the
	// demo's road bodies.
	MVSurface();

	// Sets the node group name this surface answers to.
	// p_name: node group name; compared exactly, case-sensitive.
	// Returns: nothing. Emits changed; MVSurfaceLibrary::find_surface sees the new name at once.
	void set_name(const StringName &p_name);
	// Returns: the node group name this surface answers to.
	StringName get_name() const;

	// Sets the tire stiffness, clamped to >= MIN_GRIP_VALUE.
	// p_tire_stiffness: brush model stiffness factor, unitless.
	// Returns: nothing. Emits changed; MVWheel rescales it to N/m on its next physics tick.
	void set_tire_stiffness(real_t p_tire_stiffness);
	// Returns: the tire stiffness factor, unitless.
	real_t get_tire_stiffness() const;

	// Sets the coefficient of friction, clamped to >= MIN_GRIP_VALUE.
	// p_coefficient_of_friction: multiplier on the tire's maximum force, unitless.
	// Returns: nothing. Emits changed; MVWheel reads it on its next physics tick.
	void set_coefficient_of_friction(real_t p_coefficient_of_friction);
	// Returns: the coefficient of friction, unitless.
	real_t get_coefficient_of_friction() const;

	// Sets the rolling resistance multiplier, clamped to >= MIN_GRIP_VALUE.
	// p_rolling_resistance: multiplier on the rolling resistance force, unitless.
	// Returns: nothing. Emits changed; MVWheel reads it on its next physics tick.
	void set_rolling_resistance(real_t p_rolling_resistance);
	// Returns: the rolling resistance multiplier, unitless.
	real_t get_rolling_resistance() const;

	// Sets the lateral grip assist, clamped to >= MIN_GRIP_VALUE.
	// p_lateral_grip_assist: extra lateral grip per radian of lateral slip, in 1/rad.
	// Returns: nothing. Emits changed; MVWheel reads it on its next physics tick.
	void set_lateral_grip_assist(real_t p_lateral_grip_assist);
	// Returns: the lateral grip assist, in 1/rad.
	real_t get_lateral_grip_assist() const;

	// Sets the longitudinal grip ratio, clamped to >= MIN_GRIP_VALUE.
	// p_longitudinal_grip_ratio: longitudinal grip as a fraction of lateral grip, unitless.
	// Returns: nothing. Emits changed; MVWheel reads it on its next physics tick.
	void set_longitudinal_grip_ratio(real_t p_longitudinal_grip_ratio);
	// Returns: the longitudinal grip ratio, unitless.
	real_t get_longitudinal_grip_ratio() const;
};

} // namespace godot
