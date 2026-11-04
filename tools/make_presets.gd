extends SceneTree
## Regenerates the preset resources in addons/modular_vehicles/presets/.
##
## Run from the repository root after building the extension:
##   godot --headless --path . --import
##   godot --headless --editor --path . -s tools/make_presets.gd
##
## The presets are written by ResourceSaver so their format and UIDs are Godot's own. UIDs are
## only assigned while the editor is running, hence --editor; without it the files are still
## valid but carry no uid. Only values that differ from the C++ defaults are set here, so
## regenerating after a default changes keeps the presets in sync.
##
## Terms:
##   preset: a .tres file holding one settings resource with ready-made values.
##   settings resource: an MVSurface, MVSurfaceLibrary, MVTireSettings, MVSuspensionSettings
##     or MVBrakeSettings instance.
##   surface: an MVSurface, the grip numbers for one node group (road, dirt, grass).
##   surface library: the MVSurfaceLibrary listing the surfaces a vehicle knows about.
##   default surface: the library's fallback for unknown node group names.
##   monster truck values: the per-axle overrides of addons/gevp/scenes/monster_truck.tscn,
##     identical for both axles there.

const PRESET_DIR := "res://addons/modular_vehicles/presets/"
## Frames to wait after _init so the editor and its UID registry exist before saving.
const EDITOR_SETTLE_FRAMES := 3


## Entry point: generates every preset and quits with 1 if any save failed.
## Returns: nothing. Calls quit() with the exit code the shell sees.
func _init() -> void:
	_generate_all()


## Generates every preset in dependency order (surfaces before the library that references
## them) and quits with the number of failures as the exit code.
## Returns: nothing. Writes the preset files and ends the process.
func _generate_all() -> void:
	if Engine.is_editor_hint():
		for frame in EDITOR_SETTLE_FRAMES:
			await process_frame
	DirAccess.make_dir_recursive_absolute(PRESET_DIR)

	var failures := 0
	var road: MVSurface = _make_surface("surface_road.tres", &"Road", 10.0, 3.0, 1.0, 0.05)
	var dirt: MVSurface = _make_surface("surface_dirt.tres", &"Dirt", 0.5, 2.4, 2.0, 0.0)
	var grass: MVSurface = _make_surface("surface_grass.tres", &"Grass", 0.5, 2.0, 4.0, 0.0)
	failures += _save(road, "surface_road.tres")
	failures += _save(dirt, "surface_dirt.tres")
	failures += _save(grass, "surface_grass.tres")
	failures += _save(_make_surface_library([road, dirt, grass], road), "surface_library_default.tres")

	failures += _save_defaults("tire_default.tres", &"MVTireSettings")
	failures += _save_defaults("suspension_default.tres", &"MVSuspensionSettings")
	failures += _save_defaults("brake_default.tres", &"MVBrakeSettings")
	failures += _save(_make_monster_truck_tire(), "tire_monster_truck.tres")
	failures += _save(_make_monster_truck_suspension(), "suspension_monster_truck.tres")

	if failures > 0:
		push_error("%d preset(s) failed to save" % failures)
	quit(1 if failures > 0 else 0)


## Fills one surface preset with the values of a vehicle.gd surface dictionary entry.
## file_name: preset file name inside PRESET_DIR.
## surface_name: node group name the surface answers to.
## tire_stiffness: brush model stiffness factor, unitless.
## coefficient_of_friction: multiplier on the tire's maximum force, unitless.
## rolling_resistance: multiplier on the rolling resistance force, unitless.
## lateral_grip_assist: extra lateral grip per radian of slip, in 1/rad.
## Returns: the surface, not yet saved. longitudinal_grip_ratio keeps the default 0.5.
func _make_surface(
	file_name: String,
	surface_name: StringName,
	tire_stiffness: float,
	coefficient_of_friction: float,
	rolling_resistance: float,
	lateral_grip_assist: float,
) -> MVSurface:
	var surface: MVSurface = _load_or_new(file_name, &"MVSurface")
	surface.name = surface_name
	surface.tire_stiffness = tire_stiffness
	surface.coefficient_of_friction = coefficient_of_friction
	surface.rolling_resistance = rolling_resistance
	surface.lateral_grip_assist = lateral_grip_assist
	return surface


## Saves a preset that holds nothing but the class defaults.
## file_name: preset file name inside PRESET_DIR.
## type: class name of the settings resource.
## Returns: 0 on success, 1 on failure, like _save().
func _save_defaults(file_name: String, type: StringName) -> int:
	return _save(_load_or_new(file_name, type), file_name)


## Fills the default surface library.
## surfaces: the surfaces in lookup order; saved as external references.
## default_surface: the fallback for unknown node group names.
## Returns: the library, not yet saved.
func _make_surface_library(
	surfaces: Array[MVSurface],
	default_surface: MVSurface,
) -> MVSurfaceLibrary:
	var library: MVSurfaceLibrary = _load_or_new(
		"surface_library_default.tres", &"MVSurfaceLibrary"
	)
	library.surfaces = surfaces
	library.default_surface = default_surface
	return library


## Fills the monster truck tire preset with the monster truck values.
## Returns: the tire settings, not yet saved.
func _make_monster_truck_tire() -> MVTireSettings:
	var tire: MVTireSettings = _load_or_new("tire_monster_truck.tres", &"MVTireSettings")
	tire.tire_radius = 0.85
	tire.tire_width = 1000.0
	tire.wheel_mass = 363.0
	return tire


## Fills the monster truck suspension preset with the monster truck values.
## Returns: the suspension settings, not yet saved.
func _make_monster_truck_suspension() -> MVSuspensionSettings:
	var suspension: MVSuspensionSettings = _load_or_new(
		"suspension_monster_truck.tres", &"MVSuspensionSettings"
	)
	suspension.spring_length = 0.711
	suspension.resting_ratio = 0.3
	suspension.damping_ratio = 0.2
	suspension.bump_damp_multiplier = 1.0
	suspension.rebound_damp_multiplier = 1.0
	suspension.anti_roll_ratio = 0.0
	return suspension


## Reuses an existing preset so its UID survives regeneration, or creates a fresh instance.
## file_name: preset file name inside PRESET_DIR.
## type: class name of the settings resource the preset must hold.
## Returns: a resource of that class, loaded from disk when a matching preset exists.
func _load_or_new(file_name: String, type: StringName) -> Resource:
	var path: String = PRESET_DIR + file_name
	if ResourceLoader.exists(path):
		var existing: Resource = ResourceLoader.load(path, type, ResourceLoader.CACHE_MODE_IGNORE)
		if existing != null and existing.is_class(type):
			return existing
	return ClassDB.instantiate(type)


## Saves one preset and reports the outcome on the console.
## resource: the settings resource to write.
## file_name: preset file name inside PRESET_DIR; also becomes the resource path.
## Returns: 0 on success, 1 on failure, so callers can sum failures.
func _save(resource: Resource, file_name: String) -> int:
	var path: String = PRESET_DIR + file_name
	resource.take_over_path(path)
	var error: Error = ResourceSaver.save(resource, path)
	if error != OK:
		push_error("Could not save %s: %s" % [path, error_string(error)])
		return 1
	print("Saved ", path)
	return 0
