// Modular Vehicles for Godot: aerodynamic formulas ported from addons/gevp/scripts/vehicle.gd.
//
// Terms:
//   drag: the air resistance force opposing the body's velocity, in N.
//   coefficient of drag: the unitless shape factor the owner tunes (0.3 for the demo car).
//   air density: in kg/m^3 (1.225 at sea level).
//   frontal area: the body's cross-section facing the airflow, in m^2.
#ifndef MV_AERO_HPP
#define MV_AERO_HPP

#include "mv_math.hpp"

namespace mv {

// The 1/2 of the dynamic-pressure formula 0.5 * rho * v^2 (unitless).
constexpr Real kDynamicPressureFactor = static_cast<Real>(0.5);

// Drag magnitude from the dynamic pressure, area and coefficient of drag.
// Ported from vehicle.gd process_drag(): 0.5 * air_density * speed^2 * frontal_area * cd.
// p_speed: the body's speed, in m/s; the sign does not matter since it is squared.
// p_coefficient_of_drag: the coefficient of drag, unitless.
// p_air_density: the air density, in kg/m^3.
// p_frontal_area: the frontal area, in m^2.
// Returns: the drag in N; the vehicle applies it against the normalised linear velocity.
inline Real drag_force(Real p_speed, Real p_coefficient_of_drag, Real p_air_density, Real p_frontal_area) {
	return kDynamicPressureFactor * p_air_density * p_speed * p_speed * p_frontal_area * p_coefficient_of_drag;
}

} // namespace mv

#endif // MV_AERO_HPP
