# CLAUDE.md

Guidance for AI coding agents (and humans) working in this repository.

## What this is

Modular Vehicles for Godot: a C++ GDExtension (godot-cpp, Godot 4.4+) providing raycast
rigid-body vehicle physics for any number of wheels. It is a fork of DAShoe1's Godot Easy
Vehicle Physics; the original GDScript addon under `addons/gevp/` is the reference
implementation and the source of truth for formulas and property semantics until it is
deleted (Trello card B16). The approved conversion plan and the per-branch breakdown live
on the Trello board "Modular Vehicles for Godot".

**Status:** scaffold stage. `MVVehicle` exists as an empty RigidBody3D subclass; physics is
ported branch by branch (see "Branch workflow").

## Repo map

| Path | Purpose |
|---|---|
| `addons/modular_vehicles/` | **The shippable addon.** `.gdextension`, `bin/` (compiled libs, gitignored), later `presets/`, `icons/`, `extras/` |
| `addons/gevp/` | Original GDScript addon + demos. Reference only; do not extend it. Deleted in B16 |
| `src/` | C++ sources. `register_types.*`, `mv_version.h`, `nodes/` (MVVehicle, MVWheel, ...), later `resources/`, `math/`, `editor/` |
| `src/math/` (later) | Header-only, **std-only** physics math. Never include `godot_cpp/` here; every function gets a doctest |
| `thirdparty/godot-cpp/` | Git submodule pinned to `godot-4.4.1-stable`. Never edit |
| `docs/` | Human docs: building, testing, architecture, physics, migration, api |
| `tests/` (later) | `unit/` doctest, `gut/` headless Godot scene tests |
| `project.godot` | The repo root is the demo Godot project; demo content moves to `demo/` in B14 |

Folders that Godot must not import carry an empty `.gdignore` (`src/`, `thirdparty/`, `docs/`, ...).

## Build

Prerequisites: Python 3.10+, SCons (`python -m pip install --upgrade scons`), VS 2022 Build
Tools with the C++ workload (Windows), git with submodules. First build compiles godot-cpp and
takes several minutes; later builds are incremental.

```
git submodule update --init --recursive
scons platform=windows target=editor              # what the editor loads during development
scons platform=windows target=template_debug
scons platform=windows target=template_release
scons platform=windows target=editor compiledb=yes  # compile_commands.json for clangd
scons -c platform=windows target=editor           # clean
```

Linux: `scons platform=linux target=<target>`. macOS: `scons platform=macos arch=universal target=<target>`.
Output goes to `addons/modular_vehicles/bin/libmodular_vehicles.<platform>.<target>.<arch>.<ext>`
and must match the `[libraries]` keys in `modular_vehicles.gdextension`.

Windows DLL lock: `reloadable = true` lets the editor hot-reload after a rebuild, but if a build
fails with "cannot open output file", close the editor or the running game first.

Verify a build loaded: the Output panel (or `godot --headless --path . --quit`) prints
`Modular Vehicles <version> (<target>) loaded`.

## Test

- doctest (from B03): `scons tests` then `bin\tests\mv_tests.exe`.
- GUT (from B04): `python tools/fetch_gut.py` once, then
  `godot --headless --path . -s addons/gut/gut_cmdln.gd -gdir=res://tests/gut -ginclude_subdirs -gexit`.
- Manual: open the project in Godot 4.4+ and play the demo scenes. Physics tick must be 120 Hz.

## Branch workflow (owner's rules)

- One feature branch per Trello card: `feature/<slug>`, `ci/<slug>`, `docs/<slug>`.
- Always branch from freshly fetched main: `git fetch origin && git switch -c feature/<slug> origin/main`.
- One PR per branch, opened with `gh pr create`; the owner reviews and merges every PR. Never
  push to `main`, never merge your own PR, never force-push a branch under review silently.
- Keep PRs reviewable in one sitting (roughly 200-800 lines of real change; scene files excluded).
- Move the Trello card: In Progress when the branch is pushed, In Review when the PR opens
  (put the PR link on the card), Done on merge.
- Commit messages end with the attribution trailer given in the session instructions.

## Code conventions

- Godot C++ style: tabs, `.clang-format` in the repo root, one class per `.h`/`.cpp` pair.
- All native classes use the `MV` prefix (MVVehicle, MVWheel, MVTireSettings, ...).
- `_bind_methods` order: methods, then `ADD_GROUP`/`ADD_PROPERTY` in inspector order, then
  signals, then constants. Angles use `PROPERTY_HINT_RANGE` with `radians_as_degrees`.
- Settings are `Resource` subclasses; every setter calls `emit_changed()`; nodes recompute derived
  values lazily on the next physics tick. Shared resources are never written at runtime.
- No per-tick allocations; cache `StringName`s; store colliders as `ObjectID`.
- SI units everywhere (m, kg, N, N·m, rad, rad/s); RPM only at the motor API surface.
- Physics conventions: body forward is -Z, wheel meshes face +Z, wheel local Y is up along the
  ray, slip/force vectors are `Vec2(lateral x, longitudinal y)` as in `wheel.gd`, surfaces are
  resolved from the collider's node groups.

## Do not

- Do not modify anything under `thirdparty/`.
- Do not commit files in `addons/modular_vehicles/bin/`, `bin/`, or `src/gen/`.
- Do not change `entry_symbol` or `compatibility_minimum` in the `.gdextension`.
- Do not include `godot_cpp/` headers from `src/math/`.
- Do not reintroduce `front_*`/`rear_*` axle-pair properties; the design is per-wheel + resources.
- Do not add a `plugin.cfg`; a GDExtension loads automatically.
- Do not hand-edit embedded ArrayMesh data in `.tscn` files.
- Do not rename inspector properties without updating `doc_classes/`, `docs/`, and the migration table.

## Troubleshooting

- "MSVC not found": install the VS 2022 "Desktop development with C++" workload; SCons finds it via vswhere.
- Extension fails to load: check which `[libraries]` key matched your platform/target and that the file exists.
- Editor shows stale behaviour after a rebuild: restart the editor (hot reload is best effort).
