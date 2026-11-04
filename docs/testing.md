# Testing

Two layers of tests are planned. The first, unit tests for the pure math, exists today. The
second, GUT scene tests that drive a vehicle inside a headless Godot, arrives with Trello card
B04 (see "Scene tests" below).

## Unit tests (doctest)

The physics formulas live in `src/math/*.hpp` as header-only functions in namespace `mv`.
They are exercised by [doctest](https://github.com/doctest/doctest) test cases in
`tests/unit/`, one `test_<area>.cpp` per header.

### Build and run

From the repository root (same prerequisites as [building.md](building.md), minus Godot):

```
scons tests
bin\tests\mv_tests.exe          # Windows
./bin/tests/mv_tests            # Linux / macOS
```

`scons tests` compiles `tests/unit/*.cpp` with a plain SCons environment. Only `src/` and
`thirdparty/doctest/` are on the include path and godot-cpp is neither configured nor linked,
so the tests build in a few seconds and do not need the `thirdparty/godot-cpp` submodule.
Objects go to `bin/tests/obj/`, the binary to `bin/tests/mv_tests` (`.exe` on Windows). Both
are gitignored.

Options:

| Command | Effect |
|---|---|
| `scons tests precision=double` | compiles with `MV_DOUBLE_PRECISION` (`mv::Real` = `double`), like `precision=double` does for the library. The default is single precision, which is what the shipped library uses |
| `scons -c tests` | removes the test objects and binary |
| `mv_tests --test-case="mv_tire*"` | runs the test cases whose name matches (doctest filter; `--list-test-cases` lists them) |
| `mv_tests --success` | prints every passing assertion too |
| `mv_tests --reporters=junit --out=unit_results.xml` | JUnit XML, what CI uploads as an artifact |

### Golden values

The numbers the tests check against are not derived from the C++ code. They come from
`tests/unit/golden/golden_values.gd`, a headless GDScript that evaluates the formulas copied
verbatim from the reference implementation (`addons/gevp/scripts/vehicle.gd`, `wheel.gd`) for
fixed inputs and prints the results. The last output is kept in
`tests/unit/golden/golden_values.txt`. To regenerate it:

```
godot --headless --path . --import
godot --headless --path . -s tests/unit/golden/golden_values.gd > tests/unit/golden/golden_values.txt
```

Then update the literals in `tests/unit/test_*.cpp` if anything moved. Details, including the
known case from the conversion plan (1500 kg, 0.5 weight distribution, 0.15 m spring length,
0.5 resting ratio: a per-wheel load of 3675 N gives a spring rate of 49.0 N/mm), are in
`tests/unit/golden/README.md`.

Tolerances: the GDScript computes scalars in double but stores `Vector2`/`Vector3` components
as 32-bit floats, and the C++ runs in `mv::Real` (float by default), so tests compare with
`doctest::Approx` at 1e-4 relative. The torque-table sampler is checked to within 1 % of
`Curve.sample_baked` on the demo curve at a dozen RPMs, which is the acceptance criterion of
card B03.

### The std-only rule

`src/math/` must only include the C++ standard library (`<cmath>`, `<algorithm>`, `<array>`,
`<cstdint>`), never `godot_cpp/` headers or Godot error macros. godot-cpp's wrapper types call
engine function pointers that are only bound when the library is loaded by Godot; in a
standalone test binary they are null and would crash, and linking godot-cpp into the tests would
also make them take minutes to build. Keeping the formulas std-only is what makes them testable
at all. Godot types stop at the node layer: nodes (B06 onwards) unpack `Vector3`, `Curve` and
the settings resources into plain `mv::Real` / `mv::Vec3` / `mv::TorqueTable` values and call
the math functions with those. Every public function in `src/math/` carries a comment naming
the GDScript function it was ported from.

### Continuous integration

`.github/workflows/unit-tests.yml` runs `scons tests` and the binary on `ubuntu-latest` and
`windows-latest` for every pull request and every push to `main`, and uploads
`unit_results.xml` (JUnit) per OS as an artifact. The library build itself is checked by the
separate build workflow.

## Scene tests (GUT, from card B04)

GUT tests under `tests/gut/` will load the demo scenes in a headless Godot, step physics at
120 Hz and assert on vehicle behaviour (settling height, straight-line acceleration, steering
response). Setup will be `python tools/fetch_gut.py` once, then:

```
godot --headless --path . -s addons/gut/gut_cmdln.gd -gdir=res://tests/gut -ginclude_subdirs -gexit
```

Until B04 lands, the manual check is to open the project in Godot 4.4+ and play the demo
scenes.
