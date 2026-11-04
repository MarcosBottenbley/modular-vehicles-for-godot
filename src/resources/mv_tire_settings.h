#pragma once

// Terms:
//   tire radius: distance from the wheel axis to the ground contact, in m.
//   tire width: tread width, in mm (kept from the GDScript). Reduces load sensitivity only.
//   wheel mass: mass of the spinning wheel, in kg; the wheel derives its moment of inertia
//     as 0.5 * wheel_mass * tire_radius^2.
//   brush tire model: the slip-to-force model of wheel.gd; the tire deflects like bristles
//     along a contact patch of length contact_patch.
//   contact patch: length of the ground contact in the brush tire model, in m.
//   braking grip: extra longitudinal grip while the brakes are applied, as a multiplier.
//   wheel-to-body torque: the share of longitudinal tire force fed back to the body as a torque
//     about the wheel, as a multiplier. Adds weight transfer for low centres of gravity.
//   surface multiplier: a per-tire factor on the matched MVSurface value (grip, stiffness or
//     rolling resistance), so one surface library serves tires with different compounds.

#include <godot_cpp/classes/resource.hpp>

namespace godot {

// Tire geometry and brush tire model parameters. MVWheel reads its MVTireSettings every physics
// tick; several wheels usually share one resource. An empty slot behaves like the defaults.
class MVTireSettings : public Resource {
	GDCLASS(MVTireSettings, Resource)

public:
	static constexpr real_t DEFAULT_TIRE_RADIUS = 0.3; // m
	static constexpr real_t DEFAULT_TIRE_WIDTH = 245.0; // mm
	static constexpr real_t DEFAULT_WHEEL_MASS = 15.0; // kg
	static constexpr real_t DEFAULT_CONTACT_PATCH = 0.2; // m
	static constexpr real_t DEFAULT_BRAKING_GRIP_MULTIPLIER = 1.5; // unitless
	static constexpr real_t DEFAULT_WHEEL_TO_BODY_TORQUE_MULTIPLIER = 1.0; // unitless
	static constexpr real_t DEFAULT_SURFACE_MULTIPLIER = 1.0; // unitless, grip/stiffness/rolling
	// Geometry and mass divide other quantities, so they stay strictly positive.
	static constexpr real_t MIN_TIRE_RADIUS = 0.01; // m
	static constexpr real_t MIN_TIRE_WIDTH = 1.0; // mm
	static constexpr real_t MIN_WHEEL_MASS = 0.01; // kg
	static constexpr real_t MIN_CONTACT_PATCH = 0.001; // m
	static constexpr real_t MIN_MULTIPLIER = 0.0; // unitless

private:
	real_t tire_radius = DEFAULT_TIRE_RADIUS;
	real_t tire_width = DEFAULT_TIRE_WIDTH;
	real_t wheel_mass = DEFAULT_WHEEL_MASS;
	real_t contact_patch = DEFAULT_CONTACT_PATCH;
	real_t braking_grip_multiplier = DEFAULT_BRAKING_GRIP_MULTIPLIER;
	real_t wheel_to_body_torque_multiplier = DEFAULT_WHEEL_TO_BODY_TORQUE_MULTIPLIER;
	real_t grip_multiplier = DEFAULT_SURFACE_MULTIPLIER;
	real_t stiffness_multiplier = DEFAULT_SURFACE_MULTIPLIER;
	real_t rolling_resistance_multiplier = DEFAULT_SURFACE_MULTIPLIER;

protected:
	static void _bind_methods();

public:
	// Sets the tire radius, clamped to >= MIN_TIRE_RADIUS.
	// p_tire_radius: tire radius in m.
	// Returns: nothing. Emits changed; MVWheel recomputes its ray length and moment of inertia.
	void set_tire_radius(real_t p_tire_radius);
	// Returns: the tire radius in m.
	real_t get_tire_radius() const;

	// Sets the tire width, clamped to >= MIN_TIRE_WIDTH.
	// p_tire_width: tread width in mm.
	// Returns: nothing. Emits changed; MVWheel reads it in its friction term next tick.
	void set_tire_width(real_t p_tire_width);
	// Returns: the tire width in mm.
	real_t get_tire_width() const;

	// Sets the wheel mass, clamped to >= MIN_WHEEL_MASS.
	// p_wheel_mass: mass of the spinning wheel in kg.
	// Returns: nothing. Emits changed; MVWheel recomputes its moment of inertia next tick.
	void set_wheel_mass(real_t p_wheel_mass);
	// Returns: the wheel mass in kg.
	real_t get_wheel_mass() const;

	// Sets the contact patch length, clamped to >= MIN_CONTACT_PATCH.
	// p_contact_patch: contact patch length in m.
	// Returns: nothing. Emits changed; MVWheel recomputes its cornering stiffness next tick.
	void set_contact_patch(real_t p_contact_patch);
	// Returns: the contact patch length in m.
	real_t get_contact_patch() const;

	// Sets the braking grip multiplier, clamped to >= MIN_MULTIPLIER.
	// p_braking_grip_multiplier: extra longitudinal grip while braking, unitless.
	// Returns: nothing. Emits changed; MVWheel and the vehicle's brake force solve read it.
	void set_braking_grip_multiplier(real_t p_braking_grip_multiplier);
	// Returns: the braking grip multiplier, unitless.
	real_t get_braking_grip_multiplier() const;

	// Sets the wheel-to-body torque multiplier, clamped to >= MIN_MULTIPLIER.
	// p_wheel_to_body_torque_multiplier: share of tire force fed back as body torque, unitless.
	// Returns: nothing. Emits changed; MVWheel applies the new share next tick.
	void set_wheel_to_body_torque_multiplier(real_t p_wheel_to_body_torque_multiplier);
	// Returns: the wheel-to-body torque multiplier, unitless.
	real_t get_wheel_to_body_torque_multiplier() const;

	// Sets the surface multiplier for grip, clamped to >= MIN_MULTIPLIER.
	// p_grip_multiplier: factor on MVSurface::coefficient_of_friction, unitless.
	// Returns: nothing. Emits changed; MVWheel reads it next tick.
	void set_grip_multiplier(real_t p_grip_multiplier);
	// Returns: the grip multiplier, unitless.
	real_t get_grip_multiplier() const;

	// Sets the surface multiplier for stiffness, clamped to >= MIN_MULTIPLIER.
	// p_stiffness_multiplier: factor on MVSurface::tire_stiffness, unitless.
	// Returns: nothing. Emits changed; MVWheel reads it next tick.
	void set_stiffness_multiplier(real_t p_stiffness_multiplier);
	// Returns: the stiffness multiplier, unitless.
	real_t get_stiffness_multiplier() const;

	// Sets the surface multiplier for rolling resistance, clamped to >= MIN_MULTIPLIER.
	// p_rolling_resistance_multiplier: factor on MVSurface::rolling_resistance, unitless.
	// Returns: nothing. Emits changed; MVWheel reads it next tick.
	void set_rolling_resistance_multiplier(real_t p_rolling_resistance_multiplier);
	// Returns: the rolling resistance multiplier, unitless.
	real_t get_rolling_resistance_multiplier() const;
};

} // namespace godot
