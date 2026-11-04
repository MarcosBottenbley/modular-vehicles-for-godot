// Shared helpers for the math-core unit tests (tests/unit/test_*.cpp).
//
// Terms:
//   golden value: a number printed by tests/unit/golden/golden_values.gd, which evaluates the
//     formulas copied from addons/gevp/scripts/vehicle.gd and wheel.gd in Godot. The tests
//     compare the C++ port against these, never against a reading of the C++ itself.
//   golden tolerance (kGoldenEps): the relative tolerance of a golden comparison, unitless.
//     The GDScript computes scalars in double but stores Vector2 / Vector3 components as 32-bit
//     floats, and the C++ runs in mv::Real (float by default), so exact equality is not expected.
//   Approx: doctest's approximate comparison; |a - b| <= epsilon * (1 + max(|a|, |b|)), which
//     acts relatively for large values and absolutely (epsilon) near zero.
#ifndef MV_TEST_HELPERS_HPP
#define MV_TEST_HELPERS_HPP

#include "doctest.h"

#include "math/mv_math.hpp"

// Default golden tolerance, unitless: 1e-4 relative, 1e-4 absolute near zero.
constexpr double kGoldenEps = 1e-4;

// Builds the Approx a golden comparison uses.
// p_value: the golden value, in whatever unit the test compares.
// p_epsilon: the golden tolerance, unitless; tests widen it (1 % for the torque table) only
//   where the acceptance criterion says so.
// Returns: a doctest::Approx to put on the right of == in a CHECK.
inline doctest::Approx approx(double p_value, double p_epsilon = kGoldenEps) {
	return doctest::Approx(p_value).epsilon(p_epsilon);
}

// Narrows a double literal to mv::Real so test inputs match the precision under test without
// a static_cast at every call site.
// p_value: the literal, in whatever unit the test passes on.
// Returns: the same value as mv::Real.
constexpr mv::Real R(double p_value) {
	return static_cast<mv::Real>(p_value);
}

#endif // MV_TEST_HELPERS_HPP
