#include "test_helpers.hpp"

#include "math/mv_math.hpp"

#include <type_traits>

using namespace mv;

TEST_CASE("mv_math: Real precision follows MV_DOUBLE_PRECISION") {
#ifdef MV_DOUBLE_PRECISION
	CHECK(std::is_same<Real, double>::value);
#else
	CHECK(std::is_same<Real, float>::value);
#endif
	CHECK(std::is_trivially_copyable<Vec2>::value);
	CHECK(std::is_trivially_copyable<Vec3>::value);
	CHECK(kEpsilon == approx(0.00001, 1e-9));
	CHECK(kTau == approx(2.0 * kPi, 1e-9));
}

TEST_CASE("mv_math: Vec2 arithmetic and helpers") {
	const Vec2 a(3, 4);
	const Vec2 b(-1, 2);
	CHECK((a + b) == Vec2(2, 6));
	CHECK((a - b) == Vec2(4, 2));
	CHECK((a * 2) == Vec2(6, 8));
	CHECK((2 * a) == Vec2(6, 8));
	CHECK((a / 2) == Vec2(R(1.5), 2));
	CHECK((-a) == Vec2(-3, -4));
	CHECK(a.dot(b) == approx(5.0));
	CHECK(a.length_squared() == approx(25.0));
	CHECK(a.length() == approx(5.0));
	CHECK(a.normalized().x == approx(0.6));
	CHECK(a.normalized().y == approx(0.8));
	CHECK(a.normalized().length() == approx(1.0));
	SUBCASE("normalizing the zero vector stays zero (Godot behaviour)") {
		const Vec2 z;
		CHECK(z.normalized() == Vec2(0, 0));
		CHECK(z.is_zero_approx());
		CHECK_FALSE(Vec2(R(0.001), 0).is_zero_approx());
		CHECK(Vec2(R(0.000001), R(-0.000001)).is_zero_approx());
	}
}

TEST_CASE("mv_math: Vec3 arithmetic, dot and cross") {
	const Vec3 a(1, 2, 3);
	const Vec3 b(4, 5, 6);
	CHECK((a + b) == Vec3(5, 7, 9));
	CHECK((b - a) == Vec3(3, 3, 3));
	CHECK((a * 2) == Vec3(2, 4, 6));
	CHECK((2 * a) == Vec3(2, 4, 6));
	CHECK((b / 2) == Vec3(2, R(2.5), 3));
	CHECK((-a) == Vec3(-1, -2, -3));
	CHECK(a.dot(b) == approx(32.0));
	CHECK(a.cross(b) == Vec3(-3, 6, -3));
	CHECK(Vec3(1, 0, 0).cross(Vec3(0, 1, 0)) == Vec3(0, 0, 1));
	CHECK(Vec3(2, 3, 6).length() == approx(7.0));
	CHECK(Vec3(2, 3, 6).normalized().length() == approx(1.0));
	CHECK(Vec3().normalized() == Vec3(0, 0, 0));
	CHECK(Vec3().is_zero_approx());
	CHECK_FALSE(a.is_zero_approx());
}

TEST_CASE("mv_math: scalar helpers behave like the Godot built-ins") {
	CHECK(lerp(10, 20, R(0.25)) == approx(12.5));
	CHECK(lerp(10, 20, 0) == approx(10.0));
	CHECK(lerp(10, 20, 1) == approx(20.0));

	CHECK(clamp(5, 0, 3) == 3);
	CHECK(clamp(-5, 0, 3) == 0);
	CHECK(clamp(2, 0, 3) == 2);

	CHECK(move_toward(0, 10, 3) == approx(3.0));
	CHECK(move_toward(10, 0, 3) == approx(7.0));
	CHECK(move_toward(9, 10, 3) == approx(10.0));
	CHECK(move_toward(-1, -10, 3) == approx(-4.0));

	CHECK(sign(R(2.5)) == 1);
	CHECK(sign(R(-0.1)) == -1);
	CHECK(sign(0) == 0);

	CHECK(is_zero_approx(0));
	CHECK(is_zero_approx(R(0.000001)));
	CHECK_FALSE(is_zero_approx(R(0.001)));

	CHECK(is_equal_approx(1, 1));
	CHECK(is_equal_approx(R(1000.0), R(1000.001)));
	CHECK_FALSE(is_equal_approx(R(1.0), R(1.001)));

	CHECK(deg_to_rad(180) == approx(kPi));
	CHECK(rad_to_deg(kPi / 2) == approx(90.0));
	CHECK(deg_to_rad(40) == approx(0.6981317007977318));
}
