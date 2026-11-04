# Settings resources

Wheel-side tuning lives in five `Resource` classes. A wheel (`MVWheel`, card B06) references one
of each; several wheels can share the same resource, and a wheel with an empty slot behaves as
if it held a default-constructed resource, so the defaults below are the defaults of the physics.

All units are SI (metres, kilograms, newtons, seconds, radians) except `tire_width`, which keeps
the millimetres used by the original GDScript. Angles are stored in radians and shown in degrees
in the inspector. Every setter clamps its input to the range given here, stores it and emits
`changed` only when the value actually changed; nodes pick the new values up on their next
physics tick. Shared resources are never written by the runtime.

Ready-made presets live in `addons/modular_vehicles/presets/` and are regenerated with
`tools/make_presets.gd` (see the header of that script). `tests/scripts/check_settings.gd`
verifies the classes, defaults, signals and presets headlessly.

## MVSurface

One driving surface. A wheel resolves the surface from the node groups of the collider it is
touching: the first group whose name equals `name` wins.

| Property | Default | Unit | Meaning |
|---|---|---|---|
| `name` | `"Road"` | | Node group name that selects this surface. |
| `tire_stiffness` | 10.0 | | Brush-model tire stiffness. Higher values make the tire respond faster. Scaled to N/m internally as `1e6 + 8e6 * tire_stiffness`. |
| `coefficient_of_friction` | 3.0 | | Multiplier for the force a tire can apply on this surface. |
| `rolling_resistance` | 1.0 | | Multiplier for rolling resistance on this surface. |
| `lateral_grip_assist` | 0.05 | | Extra lateral grip proportional to lateral slip. Keeps vehicles from sliding far, at the cost of realism. |
| `longitudinal_grip_ratio` | 0.5 | | Scales longitudinal grip relative to lateral grip, so wheels can spin while keeping lateral grip. |

Presets: `surface_road.tres`, `surface_dirt.tres`, `surface_grass.tres` carry the values of the
`"Road"`, `"Dirt"` and `"Grass"` entries of the original surface dictionaries.

## MVSurfaceLibrary

The set of surfaces a vehicle knows about.

| Property | Default | Unit | Meaning |
|---|---|---|---|
| `surfaces` | `[]` | | `Array[MVSurface]` searched in order by `find_surface()`. |
| `default_surface` | `null` | | Returned when no surface matches. When this is also empty, a shared built-in `MVSurface` with default (Road) values is returned, so the result of `find_surface()` is never null. |

Method: `find_surface(name: StringName) -> MVSurface`.

Preset: `surface_library_default.tres` holds Road, Dirt and Grass with Road as the default.

## MVTireSettings

| Property | Default | Unit | Meaning |
|---|---|---|---|
| `tire_radius` | 0.3 | m | Tire radius. Minimum 0.01. |
| `tire_width` | 245.0 | mm | Tire width. Does not change friction directly; it reduces tire load sensitivity. Minimum 1. |
| `wheel_mass` | 15.0 | kg | Mass of the wheel, used for its moment of inertia. Minimum 0.01. |
| `contact_patch` | 0.2 | m | Length of the contact patch in the brush tire model. Minimum 0.001. |
| `braking_grip_multiplier` | 1.5 | | Extra longitudinal grip while braking. |
| `wheel_to_body_torque_multiplier` | 1.0 | | How much of the tire's longitudinal force is also applied to the body as a torque centred on the wheel (more weight transfer with a low centre of gravity). |
| `grip_multiplier` | 1.0 | | New: scales the surface's `coefficient_of_friction` for this tire. |
| `stiffness_multiplier` | 1.0 | | New: scales the surface's `tire_stiffness` for this tire. |
| `rolling_resistance_multiplier` | 1.0 | | New: scales the surface's `rolling_resistance` for this tire. |

Presets: `tire_default.tres`; `tire_monster_truck.tres` (radius 0.85 m, width 1000 mm, mass 363 kg).

## MVSuspensionSettings

| Property | Default | Unit | Meaning |
|---|---|---|---|
| `spring_length` | 0.15 | m | Suspension travel. Minimum 0.001. |
| `spring_rate_mode` | `SPRING_RATE_AUTO_FROM_LOAD` | | `AUTO_FROM_LOAD` derives the spring rate from the static load over the wheel and `resting_ratio`; `MANUAL` uses `manual_spring_rate` and `manual_damping`. The inspector shows only the inputs of the active mode. |
| `resting_ratio` | 0.5 | | Fraction of `spring_length` the spring is compressed by at rest; 1 is fully compressed. Range 0.01 to 1. |
| `manual_spring_rate` | 49000.0 | N/m | Spring rate in manual mode. The default is what auto mode computes for a 1500 kg car with an even weight split. |
| `damping_ratio` | 0.4 | | Damping ratio used to derive the damper rates; 1 is critical damping. Passenger cars sit near 0.3, race cars up to 0.9. |
| `manual_damping` | 3400.0 | N·s/m | Base damper rate in manual mode, before the bump and rebound multipliers. |
| `bump_damp_multiplier` | 0.6667 | | Multiplier on the base damping during compression. Typical passenger car: 2/3 bump, 3/2 rebound. |
| `rebound_damp_multiplier` | 1.5 | | Multiplier on the base damping during extension. |
| `fast_damp_threshold` | 0.127 | m/s | Damper speed above which the fast (half-rate) damping applies. Was the constant `127.0` mm/s in `wheel.gd`. |
| `bump_stop_multiplier` | 1.0 | | Multiplier on the force applied when the suspension bottoms out. Lower it if the vehicle bounces off large bumps. |
| `anti_roll_ratio` | 0.25 | | Anti-roll bar stiffness as a ratio of the spring rate. |
| `camber` | 0.01745329 (1°) | rad | Camber magnitude; the wheel applies the sign for its side. Not simulated, but a small angle helps stability. Range ±45°. |
| `toe` | 0.01 (0.57°) | rad | Toe magnitude; the wheel applies the sign for its side. Range ±45°. |

Constants: `SPRING_RATE_AUTO_FROM_LOAD = 0`, `SPRING_RATE_MANUAL = 1`.

Presets: `suspension_default.tres`; `suspension_monster_truck.tres` (travel 0.711 m, resting ratio
0.3, damping ratio 0.2, bump and rebound multipliers 1.0, no anti-roll bar).

## MVBrakeSettings

| Property | Default | Unit | Meaning |
|---|---|---|---|
| `abs_enabled` | `true` | | Allows anti-lock braking on this wheel. The original script always had it on (except under handbrake). |
| `abs_pulse_time` | 0.03 | s | How long ABS releases the brake once the spin threshold is crossed. |
| `abs_spin_difference_threshold` | 12.0 | m/s | Speed difference between wheel and ground that engages ABS. Stored as a magnitude; the wheel compares against its negative like `wheel.gd` did. |
| `bias_weight` | -1.0 | | Share of the total brake force. `-1` = automatic from the static load on the wheel; `>= 0` = relative weight, normalised over all wheels of the vehicle. Minimum -1. |

Preset: `brake_default.tres`.

## Mapping from the GDScript addon

| `vehicle.gd` / `wheel.gd` | New home |
|---|---|
| `tire_stiffnesses[name]`, `coefficient_of_friction[name]`, `rolling_resistance[name]`, `lateral_grip_assist[name]`, `longitudinal_grip_ratio[name]` | `MVSurface` (one resource per dictionary key) in an `MVSurfaceLibrary` |
| `front_tire_radius` / `rear_tire_radius` | `MVTireSettings.tire_radius` |
| `front_tire_width` / `rear_tire_width` | `MVTireSettings.tire_width` |
| `front_wheel_mass` / `rear_wheel_mass` | `MVTireSettings.wheel_mass` |
| `contact_patch`, `braking_grip_multiplier`, `wheel_to_body_torque_multiplier` | `MVTireSettings` (same names) |
| `front_spring_length` / `rear_spring_length` | `MVSuspensionSettings.spring_length` |
| `front_resting_ratio` / `rear_resting_ratio` | `MVSuspensionSettings.resting_ratio` |
| `front_damping_ratio` / `rear_damping_ratio` | `MVSuspensionSettings.damping_ratio` |
| `front_bump_damp_multiplier` / `rear_bump_damp_multiplier` | `MVSuspensionSettings.bump_damp_multiplier` |
| `front_rebound_damp_multiplier` / `rear_rebound_damp_multiplier` | `MVSuspensionSettings.rebound_damp_multiplier` |
| `front_arb_ratio` / `rear_arb_ratio` | `MVSuspensionSettings.anti_roll_ratio` |
| `front_camber` / `rear_camber` | `MVSuspensionSettings.camber` |
| `front_toe` / `rear_toe` | `MVSuspensionSettings.toe` |
| `front_bump_stop_multiplier` / `rear_bump_stop_multiplier` | `MVSuspensionSettings.bump_stop_multiplier` |
| `Wheel.fast_damp_threshold` (constant 127 mm/s) | `MVSuspensionSettings.fast_damp_threshold` (0.127 m/s) |
| `front_abs_pulse_time` / `rear_abs_pulse_time` | `MVBrakeSettings.abs_pulse_time` |
| `front_abs_spin_difference_threshold` / `rear_abs_spin_difference_threshold` | `MVBrakeSettings.abs_spin_difference_threshold` |
| `front_brake_bias` | `MVBrakeSettings.bias_weight` per wheel (`-1` keeps the automatic split) |
| `front_beam_axle` / `rear_beam_axle` | Not a settings property; becomes a wheel-level option in B06 |
