// Modular Vehicles for Godot: std-only math primitives.
//
// Everything under src/math/ is header-only and must stay free of godot_cpp/ includes so the
// unit-test binary (scons tests) can link without an engine. Real, Vec2 and Vec3 mirror the
// subset of Godot's real_t / Vector2 / Vector3 that the physics formulas need. Conventions
// (see CLAUDE.md): SI units, body forward is -Z, slip and force vectors are
// Vec2(lateral x, longitudinal y) exactly as in addons/gevp/scripts/wheel.gd.
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

// Same value as Godot's CMP_EPSILON (core/math/math_defs.h).
constexpr Real kEpsilon = static_cast<Real>(0.00001);
constexpr Real kPi = static_cast<Real>(3.14159265358979323846);
constexpr Real kTau = static_cast<Real>(6.28318530717958647692);

struct Vec2 {
	Real x = 0;
	Real y = 0;

	constexpr Vec2() = default;
	constexpr Vec2(Real p_x, Real p_y) :
			x(p_x), y(p_y) {}

	constexpr Vec2 operator+(const Vec2 &p_v) const { return Vec2(x + p_v.x, y + p_v.y); }
	constexpr Vec2 operator-(const Vec2 &p_v) const { return Vec2(x - p_v.x, y - p_v.y); }
	constexpr Vec2 operator*(Real p_s) const { return Vec2(x * p_s, y * p_s); }
	constexpr Vec2 operator/(Real p_s) const { return Vec2(x / p_s, y / p_s); }
	constexpr Vec2 operator-() const { return Vec2(-x, -y); }
	constexpr bool operator==(const Vec2 &p_v) const { return x == p_v.x && y == p_v.y; }
	constexpr bool operator!=(const Vec2 &p_v) const { return !(*this == p_v); }

	constexpr Real dot(const Vec2 &p_v) const { return x * p_v.x + y * p_v.y; }
	constexpr Real length_squared() const { return x * x + y * y; }
	Real length() const { return std::sqrt(length_squared()); }
	// Godot Vector2.normalized(): the zero vector stays zero.
	Vec2 normalized() const {
		const Real l = length();
		return l > 0 ? Vec2(x / l, y / l) : Vec2(0, 0);
	}
	// Godot Vector2.is_zero_approx(): both components within kEpsilon of zero.
	bool is_zero_approx() const { return std::abs(x) < kEpsilon && std::abs(y) < kEpsilon; }
};

constexpr Vec2 operator*(Real p_s, const Vec2 &p_v) {
	return p_v * p_s;
}

struct Vec3 {
	Real x = 0;
	Real y = 0;
	Real z = 0;

	constexpr Vec3() = default;
	constexpr Vec3(Real p_x, Real p_y, Real p_z) :
			x(p_x), y(p_y), z(p_z) {}

	constexpr Vec3 operator+(const Vec3 &p_v) const { return Vec3(x + p_v.x, y + p_v.y, z + p_v.z); }
	constexpr Vec3 operator-(const Vec3 &p_v) const { return Vec3(x - p_v.x, y - p_v.y, z - p_v.z); }
	constexpr Vec3 operator*(Real p_s) const { return Vec3(x * p_s, y * p_s, z * p_s); }
	constexpr Vec3 operator/(Real p_s) const { return Vec3(x / p_s, y / p_s, z / p_s); }
	constexpr Vec3 operator-() const { return Vec3(-x, -y, -z); }
	constexpr bool operator==(const Vec3 &p_v) const { return x == p_v.x && y == p_v.y && z == p_v.z; }
	constexpr bool operator!=(const Vec3 &p_v) const { return !(*this == p_v); }

	constexpr Real dot(const Vec3 &p_v) const { return x * p_v.x + y * p_v.y + z * p_v.z; }
	constexpr Vec3 cross(const Vec3 &p_v) const {
		return Vec3(y * p_v.z - z * p_v.y, z * p_v.x - x * p_v.z, x * p_v.y - y * p_v.x);
	}
	constexpr Real length_squared() const { return x * x + y * y + z * z; }
	Real length() const { return std::sqrt(length_squared()); }
	// Godot Vector3.normalized(): the zero vector stays zero.
	Vec3 normalized() const {
		const Real l = length();
		return l > 0 ? Vec3(x / l, y / l, z / l) : Vec3(0, 0, 0);
	}
	bool is_zero_approx() const { return std::abs(x) < kEpsilon && std::abs(y) < kEpsilon && std::abs(z) < kEpsilon; }
};

constexpr Vec3 operator*(Real p_s, const Vec3 &p_v) {
	return p_v * p_s;
}

// Godot lerpf().
constexpr Real lerp(Real p_from, Real p_to, Real p_weight) {
	return p_from + (p_to - p_from) * p_weight;
}

// Godot clampf().
constexpr Real clamp(Real p_value, Real p_min, Real p_max) {
	return p_value < p_min ? p_min : (p_value > p_max ? p_max : p_value);
}

// Godot move_toward(): steps p_from towards p_to by at most p_delta without overshooting.
inline Real move_toward(Real p_from, Real p_to, Real p_delta) {
	return std::abs(p_to - p_from) <= p_delta ? p_to : p_from + (p_to - p_from < 0 ? -p_delta : p_delta);
}

// Godot signf(): -1, 0 or 1 (zero maps to zero, unlike std::copysign).
constexpr Real sign(Real p_value) {
	return p_value > 0 ? static_cast<Real>(1) : (p_value < 0 ? static_cast<Real>(-1) : static_cast<Real>(0));
}

// Godot is_zero_approx().
inline bool is_zero_approx(Real p_value) {
	return std::abs(p_value) < kEpsilon;
}

// Godot is_equal_approx(): tolerance scales with the magnitude of the operands.
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

// Godot deg_to_rad() / rad_to_deg().
constexpr Real deg_to_rad(Real p_deg) {
	return p_deg * (kPi / static_cast<Real>(180));
}
constexpr Real rad_to_deg(Real p_rad) {
	return p_rad * (static_cast<Real>(180) / kPi);
}

} // namespace mv

#endif // MV_MATH_HPP
