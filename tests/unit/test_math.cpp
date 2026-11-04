// Tests for src/math/mv_math.hpp: the Real typedef, Vec2 / Vec3 and the scalar helpers that
// mirror Godot's built-ins. These have no golden values; they check against values worked out
// by hand (3-4-5 triangles, unit vectors) and against the Godot semantics they copy.
//
// Terms:
//   Godot semantics: the behaviour of the GDScript built-in the helper is named after
//     (normalized() of a zero vector is zero, signf(0) is 0, is_zero_approx uses CMP_EPSILON).
//   plain data: trivially copyable, so the structs can live in node members and resources.
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
	const Vec2 three_four(3, 4);
	const Vec2 other(-1, 2);
	CHECK((three_four + other) == Vec2(2, 6));
	CHECK((three_four - other) == Vec2(4, 2));
	CHECK((three_four * 2) == Vec2(6, 8));
	CHECK((2 * three_four) == Vec2(6, 8));
	CHECK((three_four / 2) == Vec2(R(1.5), 2));
	CHECK((-three_four) == Vec2(-3, -4));
	CHECK(three_four.dot(other) == approx(5.0));
	CHECK(three_four.length_squared() == approx(25.0));
	CHECK(three_four.length() == approx(5.0));
	CHECK(three_four.normalized().x == approx(0.6));
	CHECK(three_four.normalized().y == approx(0.8));
	CHECK(three_four.normalized().length() == approx(1.0));
	SUBCASE("normalizing the zero vector stays zero (Godot semantics)") {
		const Vec2 zero;
		CHECK(zero.normalized() == Vec2(0, 0));
		CHECK(zero.is_zero_approx());
		CHECK_FALSE(Vec2(R(0.001), 0).is_zero_approx());
		CHECK(Vec2(R(0.000001), R(-0.000001)).is_zero_approx());
	}
}

TEST_CASE("mv_math: Vec3 arithmetic, dot and cross") {
	const Vec3 one_two_three(1, 2, 3);
	const Vec3 four_five_six(4, 5, 6);
	CHECK((one_two_three + four_five_six) == Vec3(5, 7, 9));
	CHECK((four_five_six - one_two_three) == Vec3(3, 3, 3));
	CHECK((one_two_three * 2) == Vec3(2, 4, 6));
	CHECK((2 * one_two_three) == Vec3(2, 4, 6));
	CHECK((four_five_six / 2) == Vec3(2, R(2.5), 3));
	CHECK((-one_two_three) == Vec3(-1, -2, -3));
	CHECK(one_two_three.dot(four_five_six) == approx(32.0));
	CHECK(one_two_three.cross(four_five_six) == Vec3(-3, 6, -3));
	CHECK(Vec3(1, 0, 0).cross(Vec3(0, 1, 0)) == Vec3(0, 0, 1));
	CHECK(Vec3(2, 3, 6).length() == approx(7.0));
	CHECK(Vec3(2, 3, 6).normalized().length() == approx(1.0));
	CHECK(Vec3().normalized() == Vec3(0, 0, 0));
	CHECK(Vec3().is_zero_approx());
	CHECK_FALSE(one_two_three.is_zero_approx());
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
