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

## Coding style

These rules are copied from the owner's Canyon Runner (Project Zoom) repo and apply to every
line written here, C++ and GDScript alike: the owner reviews every change by reading it, so
readability wins over cleverness and brevity.

**Readable code, in the owner's words.**
- Descriptive names. A variable says what it holds (`lateral_slip_velocity`, not `v`;
  `grounded_wheel_count`, not `n`). A function says what it does, as a verb phrase. Single-letter
  names are for loop indices and maths that mirrors a formula written in a comment next to it.
- One job per supporting function. Every function you write (not the inherited virtuals such as
  `_ready`, `_process`, `_physics_process`, `_integrate_forces`, `_notification`,
  `_bind_methods`) does one thing and its name says which. If a name needs "and" to be honest,
  split it.
- Build behaviour by composition. A virtual like `_physics_process` or a public entry point like
  `MVVehicle::rebuild()` reads as a short sequence of calls to well-named helpers, each of which
  is small enough to review on its own. No monolithic functions that read input, integrate
  physics, animate the model and emit signals in one body.
- Write maths so it reads left to right. A product whose grouping is not obvious at a glance
  (`t * t * (3.0 - 2.0 * t)` looks like t^3 and is not) gets its factors named as `const`
  locals, `t_squared * three_minus_two_t`, with the formula in a comment. Where the grouping
  itself is the point, write the parentheses out: `(height * t) * t`. In a port that must match
  the GDScript reference bit for bit, naming factors is safe and reordering them is not:
  `(3.0 - 2.0 * t) * t * t` rounds differently and moves the golden values.
- Prefer early returns and flat control flow over nested conditions. Declare locals next to their
  first use, never as members used by one method.
- A doc comment on each class states its role and how its owner uses it (`##` in GDScript, `//`
  above the class in the C++ header). Numbers that are tuning get a `const` with a name and a
  unit comment, not a literal in the maths.
- Every function you write gets a doc comment, private helpers and C++ included (`##` above it in
  GDScript, `//` above its declaration in the C++ header). It says, in this order:
  - what the function does, in a sentence or two;
  - each parameter, one line each: what it holds (with its unit) and how the function uses it;
  - `Returns:` what comes back (with its unit) and how the callers use it, or, for a function
    that returns nothing, what it changes (members, nodes, signals) and who relies on that.
  A short example:
  ```
  // Spring rate that holds the wheel's static load at the resting compression.
  // wheel_load: the static load this wheel carries, in N, from the CoG solve.
  // spring_length: full suspension travel, in m. Must be > 0; callers guard it.
  // resting_ratio: fraction of spring_length compressed at rest, 0.01..1.
  // Returns: the spring rate in N/m. MVVehicle caches it per wheel in _rebuild().
  ```
  Inline comments inside a body explain the why (the reason for a step, a pitfall); they do
  not restate the code.
- Every file you write or substantially change opens with a `Terms` block that defines each
  noun its comments use in a vehicle- or code-specific sense (static load, slip, drive share,
  differential group, resting ratio ...), so a sentence like "redistributes the group's share"
  can be read without guessing. It sits at the top of the C++ header (`src/math` headers
  included), and at the top of the class `##` doc in GDScript. One line per term: the word, then
  what it means here, with its unit if it has one. Comments then use those words in exactly that
  sense; a new noun gets a definition before use.
  ```
  // Terms:
  //   static load: the weight one wheel carries with the vehicle at rest on flat ground, in N.
  //   drive share: the fraction of motor torque a wheel receives, 0 = undriven.
  ```
- Pre-existing functions and files you only touch in passing do not need a rewrite; anything you
  add or substantially change does. Write the comments and the Terms block while writing the
  code, not as a cleanup pass.

**Godot style guide** (`docs.godotengine.org/en/stable/tutorials/scripting/gdscript/gdscript_styleguide.html`)
for the GDScript in `tests/`, `demo/` and `addons/modular_vehicles/extras/`:
- Tabs for indentation, lines under 100 characters, two blank lines between functions, one blank
  line between logical sections inside a function, LF endings, double quotes, trailing comma in
  multiline literals, `and` / `or` / `not` over symbols.
- Naming: `snake_case` files, functions, variables and signals (signals in past tense:
  `gear_changed`, `landed`); `PascalCase` classes, nodes and enum names; `CONSTANT_CASE` constants
  and enum members; a leading underscore on private members.
- Script order: `class_name`, `extends`, class `##` doc, signals, enums, constants, static vars,
  `@export` vars, other vars, `@onready` vars, static methods, built-in virtuals (`_init`,
  `_ready`, `_process`, `_physics_process`, then the rest), custom overrides, other methods, inner
  classes. Public before private in each group.
- Static typing everywhere. `:=` only when the type is obvious on the same line; write the type
  when it is not (`var gear: int = 0`, `@onready var bar: ProgressBar = $Bar`). Typed function
  signatures with `-> void` where nothing is returned.
- The same order applies to C++ classes: signals and enums, constants, exported members, other
  members, virtuals, public methods, then private helpers, with `_bind_methods` as described under
  Code conventions.

**Godot best practices** (`docs.godotengine.org/en/stable/tutorials/best_practices/index.html`):
- A scene is a class: one scripted root that uses its own nodes and adds behaviour. Game concepts
  are scenes; reusable engine-level tools are scripts or native classes.
- Scenes have no outward dependencies. A parent injects what a child needs (a node reference, a
  Callable, a NodePath) and the child reports back with signals. Signals react to events; they do
  not start behaviour. Siblings never reach into each other; the common ancestor mediates. This
  is how `MVVehicle` owns its `MVWheel` descendants and how `MVVehicleController` feeds inputs.
- No hard-coded `get_node` paths into other parts of the tree; `$Child` and `%Unique` only for a
  scene's own nodes.
- Autoloads only for systems that own their own data, need global access and run in isolation.
  Anything that edits another system's state is a regular node.
- Arrays for iteration and push / pop at the end, Dictionaries for lookup by key, a custom
  `RefCounted` class (or a `Resource` when it is tuning data) when the data needs structure or
  signals. `snake_case` folders and files, assets next to the scenes that use them.
- Hand-written `.tscn`: an exported node reference (`@export var vehicle: MVVehicle`) assigned as
  `NodePath(...)` resolves only when the node tag carries `node_paths=PackedStringArray("vehicle")`;
  without it the property is silently null. `%Name` needs `unique_name_in_owner = true`.
- The Godot editor rewrites `project.godot` on save (reflows input events, reorders sections).
  Review that diff before committing and keep intentional settings changes separate from reflows.

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
