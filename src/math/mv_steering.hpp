// Modular Vehicles for Godot: steering formulas ported from addons/gevp/scripts/vehicle.gd
// (initialize, process_steering, get_max_steering_slip_angle) and wheel.gd (steer).
#ifndef MV_STEERING_HPP
#define MV_STEERING_HPP

#include "mv_math.hpp"

namespace mv {

// Ported from vehicle.gd initialize() (front_ackermann / rear_ackermann):
//     atan(L * tan(theta) / (L - h * tan(theta))) / theta - 1
// with L the wheelbase, h half the track width (the GDScript passes `track_width * 0.5`) and
// theta the maximum steering angle in radians. Guards added for the port: tiny L, h or theta
// return 0; when the denominator reaches zero or goes negative the inner wheel would steer
// past 90 degrees, so the correction saturates at `(pi/2) / theta - 1`.
inline Real ackermann_correction(Real p_wheelbase, Real p_half_track, Real p_max_steering_angle) {
	if (p_wheelbase <= kEpsilon || p_half_track <= kEpsilon || p_max_steering_angle <= kEpsilon) {
		return 0;
	}
	const Real tan_theta = std::tan(p_max_steering_angle);
	const Real d = p_wheelbase - p_half_track * tan_theta;
	if (d <= static_cast<Real>(1e-4)) {
		return (kPi / static_cast<Real>(2)) / p_max_steering_angle - static_cast<Real>(1);
	}
	return std::atan(p_wheelbase * tan_theta / d) / p_max_steering_angle - static_cast<Real>(1);
}

// Ported from wheel.gd steer(): the wheel's yaw in radians for a steering input in -1..1.
// `ackermann` is the per-wheel correction (positive on one side, negated on the other).
inline Real steering_angle(Real p_input, Real p_steering_ratio, Real p_max_steering_angle, Real p_ackermann, Real p_toe) {
	const Real input = p_input * p_steering_ratio;
	return (p_max_steering_angle * (input + (static_cast<Real>(1) - std::cos(input * static_cast<Real>(0.5) * kPi)) * p_ackermann)) + p_toe;
}

// Ported from vehicle.gd process_steering(): `pow(abs(amount), exponent) * sign(amount)`.
inline Real steering_exponent_adjust(Real p_steering_amount, Real p_steering_exponent) {
	return std::pow(std::abs(p_steering_amount), p_steering_exponent) * sign(p_steering_amount);
}

// Ported from vehicle.gd process_steering(): the steering speed scaled by vehicle speed and the
// maximum steering angle. Returns 0 when speed, decay or max angle would divide by zero (the
// GDScript yields inf there, which the subsequent clamps absorb; 0 is the safe equivalent).
inline Real steering_speed_correction(Real p_steering_speed, Real p_speed, Real p_steering_speed_decay, Real p_max_steering_angle) {
	const Real denominator = p_speed * p_steering_speed_decay * p_max_steering_angle;
	if (is_zero_approx(denominator)) {
		return 0;
	}
	return p_steering_speed / denominator;
}

// Ported from vehicle.gd process_steering(): the countersteer assist term before the
// low-speed / reversing cutoff. local_velocity_normalized_x is `local_velocity.normalized().x`.
inline Real countersteer_correction(Real p_steering_adjust, Real p_local_velocity_normalized_x, Real p_max_steering_angle, Real p_countersteer_assist) {
	const Real asin_x = std::asin(clamp(p_local_velocity_normalized_x, -1, 1));
	return (static_cast<Real>(1) - std::abs(p_steering_adjust)) * clamp(asin_x, -p_max_steering_angle, p_max_steering_angle) * p_countersteer_assist;
}

// Ported from vehicle.gd get_max_steering_slip_angle(): fold helper that keeps whichever
// lateral slip angle has the larger magnitude (sign preserved). Start the fold from 0.
inline Real dominant_slip_angle(Real p_current, Real p_candidate) {
	return std::abs(p_current) < std::abs(p_candidate) ? p_candidate : p_current;
}

} // namespace mv

#endif // MV_STEERING_HPP
