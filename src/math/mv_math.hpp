// Modular Vehicles for Godot: std-only math primitives shared by every header in src/math/.
//
// Everything under src/math/ is header-only and must stay free of godot_cpp/ includes so the
// unit-test binary (scons tests) links without an engine. Real, Vec2 and Vec3 mirror the subset
// of Godot's real_t / Vector2 / Vector3 that the physics formulas need. Conventions (CLAUDE.md):
// SI units, body forward is -Z, slip and force vectors are Vec2(lateral x, longitudinal y)
// exactly as in addons/gevp/scripts/wheel.gd.
//
// Terms:
//   Real: the scalar type of the whole math layer; float, or double when MV_DOUBLE_PRECISION is
//     defined (the SConstruct defines it for precision=double builds).
//   epsilon (kEpsilon): the smallest magnitude this layer treats as non-zero, unitless, equal
//     to Godot's CMP_EPSILON so "approximately zero" means the same here and in GDScript.
//   approximately zero / approximately equal: within epsilon, as Godot's is_zero_approx() and
//     is_equal_approx() define it.
//   zero guard: the Godot behaviour of returning a zero vector from normalized() on a zero
//     vector instead of dividing by zero.
//   component-wise: applied to x, y (and z) independently.
//   reference / GDScript: addons/gevp/scripts/vehicle.gd and wheel.gd, the source of truth the
//     formulas are ported from.
#ifndef MV_MATH_HPP
#define MV_MATH_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mv {

#ifdef MV_DOUBLE_PRECISION
using Real = double;
#else
using Real = float;
#endif

// Same value as Godot's CMP_EPSILON (core/math/math_defs.h), unitless.
constexpr Real kEpsilon = static_cast<Real>(0.00001);
// Pi and 2*pi in radians.
constexpr Real kPi = static_cast<Real>(3.14159265358979323846);
constexpr Real kTau = static_cast<Real>(6.28318530717958647692);
// Degrees per radian, used by the angle conversions below.
constexpr Real kDegreesPerHalfTurn = static_cast<Real>(180);

// Two-component vector in Real. Used for slip and force vectors (x lateral, y longitudinal)
// and for the planar part of a wheel's local velocity. Plain data: no constructor side effects,
// trivially copyable, safe to store in node members and resources.
struct Vec2 {
	Real x = 0;
	Real y = 0;

	constexpr Vec2() = default;
	// p_x, p_y: the two components, in whatever unit the caller's vector carries.
	constexpr Vec2(Real p_x, Real p_y) :
			x(p_x), y(p_y) {}

	// Component-wise sum.
	// p_v: the vector to add, same unit as this one.
	// Returns: this + p_v.
	constexpr Vec2 operator+(const Vec2 &p_v) const { return Vec2(x + p_v.x, y + p_v.y); }
	// Component-wise difference.
	// p_v: the vector to subtract, same unit as this one.
	// Returns: this - p_v.
	constexpr Vec2 operator-(const Vec2 &p_v) const { return Vec2(x - p_v.x, y - p_v.y); }
	// Uniform scale.
	// p_s: unitless factor applied to both components.
	// Returns: this * p_s.
	constexpr Vec2 operator*(Real p_s) const { return Vec2(x * p_s, y * p_s); }
	// Uniform division; callers guard p_s against zero.
	// p_s: unitless divisor applied to both components.
	// Returns: this / p_s.
	constexpr Vec2 operator/(Real p_s) const { return Vec2(x / p_s, y / p_s); }
	// Negation.
	// Returns: the vector pointing the opposite way.
	constexpr Vec2 operator-() const { return Vec2(-x, -y); }
	// Exact component equality (no epsilon); tests use it on integer-valued vectors.
	// p_v: the vector to compare with.
	// Returns: true when both components are bit-equal.
	constexpr bool operator==(const Vec2 &p_v) const { return x == p_v.x && y == p_v.y; }
	// Exact component inequality.
	// p_v: the vector to compare with.
	// Returns: true when any component differs.
	constexpr bool operator!=(const Vec2 &p_v) const { return !(*this == p_v); }

	// Dot product.
	// p_v: the other vector, same unit as this one.
	// Returns: x*x' + y*y', in the product of the two units.
	constexpr Real dot(const Vec2 &p_v) const { return x * p_v.x + y * p_v.y; }
	// Squared Euclidean length, cheaper than length() when only comparing magnitudes.
	// Returns: x^2 + y^2, in the unit squared.
	constexpr Real length_squared() const { return x * x + y * y; }
	// Euclidean length.
	// Returns: sqrt(x^2 + y^2), in the vector's unit.
	Real length() const { return std::sqrt(length_squared()); }
	// Unit-length copy with Godot's zero guard: the zero vector stays zero.
	// Returns: this / length(), or (0, 0) when the length is 0.
	Vec2 normalized() const {
		const Real vector_length = length();
		return vector_length > 0 ? Vec2(x / vector_length, y / vector_length) : Vec2(0, 0);
	}
	// Godot Vector2.is_zero_approx(): both components within epsilon of zero.
	// Returns: true when |x| < kEpsilon and |y| < kEpsilon.
	bool is_zero_approx() const { return std::abs(x) < kEpsilon && std::abs(y) < kEpsilon; }
};

// Scalar-first scale, so `2 * v` reads like maths.
// p_s: unitless factor.
// p_v: the vector to scale.
// Returns: p_v * p_s.
constexpr Vec2 operator*(Real p_s, const Vec2 &p_v) {
	return p_v * p_s;
}

// Three-component vector in Real. Used for a wheel's local velocity (x right, y up, z back in
// the ray's frame) and positions. Plain data, trivially copyable; the nodes convert from
// godot::Vector3 at the boundary.
struct Vec3 {
	Real x = 0;
	Real y = 0;
	Real z = 0;

	constexpr Vec3() = default;
	// p_x, p_y, p_z: the three components, in whatever unit the caller's vector carries.
	constexpr Vec3(Real p_x, Real p_y, Real p_z) :
			x(p_x), y(p_y), z(p_z) {}

	// Component-wise sum.
	// p_v: the vector to add, same unit as this one.
	// Returns: this + p_v.
	constexpr Vec3 operator+(const Vec3 &p_v) const { return Vec3(x + p_v.x, y + p_v.y, z + p_v.z); }
	// Component-wise difference.
	// p_v: the vector to subtract, same unit as this one.
	// Returns: this - p_v.
	constexpr Vec3 operator-(const Vec3 &p_v) const { return Vec3(x - p_v.x, y - p_v.y, z - p_v.z); }
	// Uniform scale.
	// p_s: unitless factor applied to all components.
	// Returns: this * p_s.
	constexpr Vec3 operator*(Real p_s) const { return Vec3(x * p_s, y * p_s, z * p_s); }
	// Uniform division; callers guard p_s against zero.
	// p_s: unitless divisor applied to all components.
	// Returns: this / p_s.
	constexpr Vec3 operator/(Real p_s) const { return Vec3(x / p_s, y / p_s, z / p_s); }
	// Negation.
	// Returns: the vector pointing the opposite way.
	constexpr Vec3 operator-() const { return Vec3(-x, -y, -z); }
	// Exact component equality (no epsilon); tests use it on integer-valued vectors.
	// p_v: the vector to compare with.
	// Returns: true when all components are bit-equal.
	constexpr bool operator==(const Vec3 &p_v) const { return x == p_v.x && y == p_v.y && z == p_v.z; }
	// Exact component inequality.
	// p_v: the vector to compare with.
	// Returns: true when any component differs.
	constexpr bool operator!=(const Vec3 &p_v) const { return !(*this == p_v); }

	// Dot product.
	// p_v: the other vector, same unit as this one.
	// Returns: x*x' + y*y' + z*z', in the product of the two units.
	constexpr Real dot(const Vec3 &p_v) const { return x * p_v.x + y * p_v.y + z * p_v.z; }
	// Right-handed cross product, same handedness as Godot.
	// p_v: the other vector, same unit as this one.
	// Returns: the vector perpendicular to both, in the product of the two units.
	constexpr Vec3 cross(const Vec3 &p_v) const {
		return Vec3(y * p_v.z - z * p_v.y, z * p_v.x - x * p_v.z, x * p_v.y - y * p_v.x);
	}
	// Squared Euclidean length, cheaper than length() when only comparing magnitudes.
	// Returns: x^2 + y^2 + z^2, in the unit squared.
	constexpr Real length_squared() const { return x * x + y * y + z * z; }
	// Euclidean length.
	// Returns: sqrt(x^2 + y^2 + z^2), in the vector's unit.
	Real length() const { return std::sqrt(length_squared()); }
	// Unit-length copy with Godot's zero guard: the zero vector stays zero.
	// Returns: this / length(), or (0, 0, 0) when the length is 0.
	Vec3 normalized() const {
		const Real vector_length = length();
		return vector_length > 0 ? Vec3(x / vector_length, y / vector_length, z / vector_length) : Vec3(0, 0, 0);
	}
	// Godot Vector3.is_zero_approx(): every component within epsilon of zero.
	// Returns: true when |x|, |y| and |z| are all < kEpsilon.
	bool is_zero_approx() const { return std::abs(x) < kEpsilon && std::abs(y) < kEpsilon && std::abs(z) < kEpsilon; }
};

// Scalar-first scale, so `2 * v` reads like maths.
// p_s: unitless factor.
// p_v: the vector to scale.
// Returns: p_v * p_s.
constexpr Vec3 operator*(Real p_s, const Vec3 &p_v) {
	return p_v * p_s;
}

// Godot lerpf(): linear interpolation, not clamped.
// p_from: value at weight 0, any unit.
// p_to: value at weight 1, same unit.
// p_weight: unitless position between them; 0..1 interpolates, outside extrapolates.
// Returns: p_from + (p_to - p_from) * p_weight, in the inputs' unit.
constexpr Real lerp(Real p_from, Real p_to, Real p_weight) {
	return p_from + (p_to - p_from) * p_weight;
}

// Godot clampf(): limits a value to a closed range.
// p_value: the value to limit, any unit.
// p_min, p_max: the range bounds, same unit; callers keep p_min <= p_max.
// Returns: p_value pulled into [p_min, p_max].
constexpr Real clamp(Real p_value, Real p_min, Real p_max) {
	return p_value < p_min ? p_min : (p_value > p_max ? p_max : p_value);
}

// Godot move_toward(): steps a value towards a target without overshooting. Used for input
// smoothing (steering, throttle, brake amounts) in the nodes.
// p_from: the current value, any unit.
// p_to: the target, same unit.
// p_delta: the largest step allowed this call, same unit, >= 0.
// Returns: p_to when it is within p_delta, else p_from moved p_delta towards it.
inline Real move_toward(Real p_from, Real p_to, Real p_delta) {
	return std::abs(p_to - p_from) <= p_delta ? p_to : p_from + (p_to - p_from < 0 ? -p_delta : p_delta);
}

// Godot signf(): the sign as -1, 0 or 1. Zero maps to zero, unlike std::copysign, which the
// tire model relies on when a velocity component is exactly zero.
// p_value: any real, any unit.
// Returns: -1, 0 or 1, unitless.
constexpr Real sign(Real p_value) {
	return p_value > 0 ? static_cast<Real>(1) : (p_value < 0 ? static_cast<Real>(-1) : static_cast<Real>(0));
}

// Godot is_zero_approx().
// p_value: any real, any unit.
// Returns: true when |p_value| < kEpsilon.
inline bool is_zero_approx(Real p_value) {
	return std::abs(p_value) < kEpsilon;
}

// Godot is_equal_approx(): approximate equality whose tolerance scales with the magnitude of
// the operands, so large values are compared relatively and small ones absolutely.
// p_a, p_b: the two values to compare, same unit.
// Returns: true when they are equal within kEpsilon * |p_a| (at least kEpsilon).
inline bool is_equal_approx(Real p_a, Real p_b) {
	if (p_a == p_b) {
		return true;
	}
	Real tolerance = kEpsilon * std::abs(p_a);
	if (tolerance < kEpsilon) {
		tolerance = kEpsilon;
	}
	return std::abs(p_a - p_b) < tolerance;
}

// Godot deg_to_rad().
// p_deg: an angle in degrees.
// Returns: the same angle in radians.
constexpr Real deg_to_rad(Real p_deg) {
	return p_deg * (kPi / kDegreesPerHalfTurn);
}

// Godot rad_to_deg().
// p_rad: an angle in radians.
// Returns: the same angle in degrees.
constexpr Real rad_to_deg(Real p_rad) {
	return p_rad * (kDegreesPerHalfTurn / kPi);
}

} // namespace mv

#endif // MV_MATH_HPP
