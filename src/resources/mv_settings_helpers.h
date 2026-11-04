#pragma once

// Terms:
//   settings resource: one of the MV*Settings / MVSurface Resource subclasses that hold wheel
//     tuning. Shared between wheels and never written by the runtime.
//   range hint: the PROPERTY_HINT_RANGE string "min,max,step[,flags]" that shapes an
//     inspector slider. The minimum is also the value the matching setter clamps to.
//   inspector maximum: the upper end of a slider. Presentation only; "or_greater" lets the
//     user type past it and the setter does not clamp to it.

#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/variant.hpp>

namespace godot {

// Stores a new value in a settings member and reports whether it differed from the old one,
// so every setter emits changed exactly once per real change.
// r_member: the member being written.
// p_value: the new value, already clamped by the caller.
// Returns: true when r_member changed; the caller then calls emit_changed().
template <typename T>
inline bool mv_store_if_changed(T &r_member, const T &p_value) {
	if (r_member == p_value) {
		return false;
	}
	r_member = p_value;
	return true;
}

// Decimals kept when a range hint number is printed. The constants are real_t, so printing
// them as doubles would show float noise such as 0.00999999977648; four decimals cover the
// finest slider step used (0.0001) and hide the noise.
constexpr int MV_RANGE_HINT_DECIMALS = 4;

// Builds a range hint from numbers so the minimum the inspector shows and the minimum the
// setter clamps to come from the same named constant.
// p_min: slider minimum, in the property's unit.
// p_max: inspector maximum, in the property's unit.
// p_step: slider step, in the property's unit; no finer than 0.0001.
// p_flags: extra comma-separated hint flags such as "or_greater,suffix:m"; may be empty.
// Returns: the hint string passed to ADD_PROPERTY.
inline String mv_range_hint(double p_min, double p_max, double p_step, const String &p_flags = String()) {
	String hint = String::num(p_min, MV_RANGE_HINT_DECIMALS) + "," + String::num(p_max, MV_RANGE_HINT_DECIMALS) + "," + String::num(p_step, MV_RANGE_HINT_DECIMALS);
	if (!p_flags.is_empty()) {
		hint += "," + p_flags;
	}
	return hint;
}

} // namespace godot
