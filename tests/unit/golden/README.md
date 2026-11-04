# Golden values for the math-core unit tests

`golden_values.gd` evaluates the formulas of the reference GDScript implementation
(`addons/gevp/scripts/vehicle.gd` and `wheel.gd`) for a fixed set of inputs and prints the
results. The numbers pasted into `tests/unit/test_*.cpp` come from this script, never from a
reading of the C++ port, so a test failure means the port drifted from the reference.

`golden_values.txt` is the last output, kept so a rerun can be diffed.

## Regenerate

From the repository root, with a Godot 4.4+ binary (the file in the repo was produced with
Godot 4.7.2):

```
godot --headless --path . --import
godot --headless --path . -s tests/unit/golden/golden_values.gd > tests/unit/golden/golden_values.txt
```

The first command imports the project once (needed so the script can load the demo scene that
holds the torque curve). It may rewrite `.import` files under `addons/gevp/` with newer import
options; revert those before committing. Errors about the missing `modular_vehicles` library
are expected when the extension has not been built and do not affect the values; strip them
from the saved text file.

## What is printed

| Section | Reference function | Used by |
|---|---|---|
| `[suspension]` | `calculate_spring_rate`, `calculate_damping`, `calculate_axle_spring_force`, the damping / bottom-out / max-length blocks of `process_suspension` | `test_suspension.cpp` |
| `[tire.*]` | `process_tires` + `process_rolling_resistance` with every intermediate | `test_tire.cpp` |
| `[drivetrain]` | `get_gear_ratio`, `get_torque_at_rpm` (and a 128-sample bake of the demo `Curve`), `process_motor`, `process_clutch` | `test_drivetrain.cpp` |
| `[steering]` | Ackermann term of `initialize`, `wheel.gd steer`, `process_steering` helpers, `get_max_steering_slip_angle` | `test_steering.cpp` |
| `[aero]` | `process_drag` | `test_aero.cpp` |

The known case from the conversion plan is `known_case.*`: 1500 kg, 0.5 front weight
distribution, 0.15 m spring length, 0.5 resting ratio. `vehicle.gd` passes
`1500 * 0.5 * 4.9 = 3675 N` as the per-wheel load, which gives a spring rate of 49.0 N/mm.

## Rules

- Copy formulas verbatim from the GDScript; turn node state into parameters, change nothing
  else. If a formula in the reference changes, update the copy and regenerate.
- Scalars are doubles in GDScript but `Vector2` / `Vector3` components are 32-bit floats, which
  is why some printed values carry float rounding (for example `-1416.5465087891`). The C++
  tests compare with a relative tolerance of 1e-4 (1 % for the torque-table interpolation).
