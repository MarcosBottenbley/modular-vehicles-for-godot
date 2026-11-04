#include "test_helpers.hpp"

#include "math/mv_aero.hpp"

using namespace mv;

// Golden values: tests/unit/golden/golden_values.txt, [aero].
TEST_CASE("mv_aero: drag_force matches vehicle.gd process_drag()") {
	CHECK(drag_force(20, R(0.3), R(1.225), 2) == approx(147.0));
	CHECK(drag_force(R(55.5), R(0.28), R(1.225), R(2.2)) == approx(1162.178325));
	CHECK(drag_force(0, R(0.3), R(1.225), 2) == approx(0.0));
	SUBCASE("drag grows with the square of speed") {
		CHECK(drag_force(40, R(0.3), R(1.225), 2) == approx(4.0 * 147.0));
	}
	SUBCASE("direction is the caller's business: negative speed still yields a positive magnitude") {
		CHECK(drag_force(-20, R(0.3), R(1.225), 2) == approx(147.0));
	}
}
