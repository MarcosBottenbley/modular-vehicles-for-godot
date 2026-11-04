// Modular Vehicles for Godot: aerodynamic formulas ported from addons/gevp/scripts/vehicle.gd.
#ifndef MV_AERO_HPP
#define MV_AERO_HPP

#include "mv_math.hpp"

namespace mv {

// Ported from vehicle.gd process_drag(): `0.5 * air_density * speed^2 * frontal_area * cd`.
// Returns the drag magnitude in N; the caller applies it against the velocity direction.
inline Real drag_force(Real p_speed, Real p_coefficient_of_drag, Real p_air_density, Real p_frontal_area) {
	return static_cast<Real>(0.5) * p_air_density * p_speed * p_speed * p_frontal_area * p_coefficient_of_drag;
}

} // namespace mv

#endif // MV_AERO_HPP
