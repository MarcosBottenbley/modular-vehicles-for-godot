// Modular Vehicles for Godot: steering formulas ported from addons/gevp/scripts/vehicle.gd
// (initialize, process_steering, get_max_steering_slip_angle) and wheel.gd (steer).
//
// Terms:
//   steering input: the driver's smoothed steering, unitless -1..1 (negative is right in the
//     GDScript's convention, which the wheel yaw follows).
//   steering ratio: how much of the steering input a wheel receives, unitless (1 for a front
//     wheel, 0 for a fixed rear wheel).
//   maximum steering angle (theta): the wheel yaw at full input, in rad.
//   wheelbase (L): the distance between the front and rear axles, in m.
//   half track (h): half the distance between the two wheels of an axle, in m; vehicle.gd
//     passes `track_width * 0.5`.
//   Ackermann correction: the unitless per-wheel term that turns the inner wheel more than the
//     outer one; positive on one side of the axle, negated on the other.
//   toe: the static yaw offset of a wheel, in rad.
//   steering exponent: the unitless power applied to the steering input so it slows near the
//     limits (1.5 in the demo).
//   steering speed correction: the rate the steering input may change this tick, in input
//     units per second, slowed by vehicle speed.
//   countersteer correction: the input added to steer towards the direction of travel,
//     unitless, from the countersteer assist factor.
//   slip angle: a wheel's lateral slip (slip.x of the tire model), in rad.
//   dominant slip angle: the front-axle slip angle with the larger magnitude, which gates the
//     steering assist.
#ifndef MV_STEERING_HPP
#define MV_STEERING_HPP

#include "mv_math.hpp"

namespace mv {

// Below this denominator, in m, the Ackermann formula saturates (guard added for the port).
constexpr Real kAckermannDenominatorCutoff = static_cast<Real>(1e-4);
// wheel.gd steer() blends the Ackermann term with 1 - cos(input * 0.5 * pi) (unitless factor).
constexpr Real kAckermannBlendHalf = static_cast<Real>(0.5);

// Ackermann correction for an axle.
// Ported from vehicle.gd initialize() (front_ackermann / rear_ackermann):
//     atan(L * tan(theta) / (L - h * tan(theta))) / theta - 1
// Guards added for the port: tiny L, h or theta return 0; when the denominator reaches zero or
// goes negative the inner wheel would steer past 90 degrees, so the correction saturates at
// `(pi/2) / theta - 1`.
// p_wheelbase: the wheelbase L, in m.
// p_half_track: the half track h, in m.
// p_max_steering_angle: the maximum steering angle theta, in rad.
// Returns: the Ackermann correction, unitless; the vehicle stores +value on one wheel of the
// axle and -value on the other, and steering_angle() applies it.
inline Real ackermann_correction(Real p_wheelbase, Real p_half_track, Real p_max_steering_angle) {
	if (p_wheelbase <= kEpsilon || p_half_track <= kEpsilon || p_max_steering_angle <= kEpsilon) {
		return 0;
	}
	const Real tan_theta = std::tan(p_max_steering_angle);
	const Real denominator = p_wheelbase - p_half_track * tan_theta;
	if (denominator <= kAckermannDenominatorCutoff) {
		return (kPi / static_cast<Real>(2)) / p_max_steering_angle - static_cast<Real>(1);
	}
	return std::atan(p_wheelbase * tan_theta / denominator) / p_max_steering_angle - static_cast<Real>(1);
}

// A wheel's yaw for a steering input.
// Ported from wheel.gd steer():
//     rotation.y = max_steering_angle * (input + (1 - cos(input * 0.5 * PI)) * ackermann) + toe
// p_input: the steering input, unitless -1..1.
// p_steering_ratio: the wheel's steering ratio, unitless.
// p_max_steering_angle: the maximum steering angle, in rad.
// p_ackermann: this wheel's signed Ackermann correction, unitless.
// p_toe: this wheel's toe, in rad.
// Returns: the wheel yaw in rad; the wheel node sets its rotation.y to it.
inline Real steering_angle(Real p_input, Real p_steering_ratio, Real p_max_steering_angle, Real p_ackermann, Real p_toe) {
	const Real wheel_input = p_input * p_steering_ratio;
	const Real ackermann_blend = static_cast<Real>(1) - std::cos(wheel_input * kAckermannBlendHalf * kPi);
	return (p_max_steering_angle * (wheel_input + ackermann_blend * p_ackermann)) + p_toe;
}

// Steering input raised to the steering exponent, sign preserved.
// Ported from vehicle.gd process_steering(): pow(abs(amount), exponent) * sign(amount).
// p_steering_amount: the steering input, unitless -1..1.
// p_steering_exponent: the steering exponent, unitless.
// Returns: the adjusted input, unitless -1..1; the vehicle adds the countersteer correction to
// it and steers every wheel with the sum.
inline Real steering_exponent_adjust(Real p_steering_amount, Real p_steering_exponent) {
	return std::pow(std::abs(p_steering_amount), p_steering_exponent) * sign(p_steering_amount);
}

// The steering speed correction: the input rate scaled down by vehicle speed and the maximum
// steering angle.
// Ported from vehicle.gd process_steering(): steering_speed / (speed * decay) / max_angle.
// Guard added for the port: a zero denominator returns 0 (the GDScript yields inf there, which
// its subsequent clamps absorb; 0 is the safe equivalent for a pure function).
// p_steering_speed: the owner's steering speed, in input units per second.
// p_speed: the vehicle speed, in m/s.
// p_steering_speed_decay: the owner's decay factor, unitless.
// p_max_steering_angle: the maximum steering angle, in rad.
// Returns: the steering speed correction in input units per second; the vehicle multiplies it
// by delta to step the steering input.
inline Real steering_speed_correction(Real p_steering_speed, Real p_speed, Real p_steering_speed_decay, Real p_max_steering_angle) {
	const Real denominator = p_speed * p_steering_speed_decay * p_max_steering_angle;
	if (is_zero_approx(denominator)) {
		return 0;
	}
	return p_steering_speed / denominator;
}

// The countersteer correction before the low-speed / reversing cut-off.
// Ported from vehicle.gd process_steering():
//     (1 - abs(adjust)) * clamp(asin(local_velocity.normalized().x), -max, max) * assist
// p_steering_adjust: from steering_exponent_adjust(), unitless.
// p_local_velocity_normalized_x: the x component of the body's normalised local velocity,
//   unitless -1..1 (clamped before asin so rounding cannot produce NaN).
// p_max_steering_angle: the maximum steering angle, in rad.
// p_countersteer_assist: the owner's countersteer assist factor, unitless.
// Returns: the countersteer correction in rad; the vehicle divides it by -max angle and scales
// it before adding it to the steering input.
inline Real countersteer_correction(Real p_steering_adjust, Real p_local_velocity_normalized_x, Real p_max_steering_angle, Real p_countersteer_assist) {
	const Real travel_angle = std::asin(clamp(p_local_velocity_normalized_x, -1, 1));
	const Real headroom = static_cast<Real>(1) - std::abs(p_steering_adjust);
	return headroom * clamp(travel_angle, -p_max_steering_angle, p_max_steering_angle) * p_countersteer_assist;
}

// Fold step for the dominant slip angle: keeps whichever slip angle has the larger magnitude,
// sign preserved. Start the fold from 0 and feed every front wheel.
// Ported from vehicle.gd get_max_steering_slip_angle().
// p_current: the dominant slip angle so far, in rad.
// p_candidate: the next wheel's slip angle, in rad.
// Returns: the new dominant slip angle in rad; the vehicle compares it with the steering slip
// assist threshold.
inline Real dominant_slip_angle(Real p_current, Real p_candidate) {
	return std::abs(p_current) < std::abs(p_candidate) ? p_candidate : p_current;
}

} // namespace mv

#endif // MV_STEERING_HPP
