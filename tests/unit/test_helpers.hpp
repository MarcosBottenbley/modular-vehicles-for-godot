// Shared helpers for the math-core unit tests.
//
// Golden values come from tests/unit/golden/golden_values.gd, which evaluates the formulas
// copied from addons/gevp/scripts/vehicle.gd and wheel.gd in Godot (scalars in double,
// Vector2/Vector3 components in float). The C++ runs in mv::Real (float by default), so the
// comparisons use a relative tolerance instead of exact equality.
#ifndef MV_TEST_HELPERS_HPP
#define MV_TEST_HELPERS_HPP

#include "doctest.h"

#include "math/mv_math.hpp"

// Default tolerance: 1e-4 relative (plus 1e-4 absolute near zero via doctest's scale).
constexpr double kGoldenEps = 1e-4;

inline doctest::Approx approx(double p_value, double p_epsilon = kGoldenEps) {
	return doctest::Approx(p_value).epsilon(p_epsilon);
}

constexpr mv::Real R(double p_value) {
	return static_cast<mv::Real>(p_value);
}

#endif // MV_TEST_HELPERS_HPP
