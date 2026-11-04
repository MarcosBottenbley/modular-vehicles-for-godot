# Regenerates the preset resources in addons/modular_vehicles/presets/.
#
# Run from the repository root after building the extension:
#   godot --headless --path . --import
#   godot --headless --editor --path . -s tools/make_presets.gd
#
# The presets are written by ResourceSaver so their format and UIDs are Godot's own. UIDs are
# only assigned while the editor is running, hence --editor; without it the files are still
# valid but carry no uid. Defaults come from the C++ classes; only values that differ from the
# defaults are set here, so regenerating after a default changes keeps the presets in sync.
extends SceneTree

const PRESET_DIR := "res://addons/modular_vehicles/presets/"


func _init() -> void:
	_generate()


func _generate() -> void:
	if Engine.is_editor_hint():
		# The editor (and its UID registry) is created after this script; let it settle first.
		for i in 3:
			await process_frame
	var errors := 0
	DirAccess.make_dir_recursive_absolute(PRESET_DIR)

	# Surfaces mirror the "Road", "Dirt" and "Grass" dictionaries of vehicle.gd.
	var road: MVSurface = _load_or_new(PRESET_DIR + "surface_road.tres", "MVSurface")
	road.name = &"Road"
	road.tire_stiffness = 10.0
	road.coefficient_of_friction = 3.0
	road.rolling_resistance = 1.0
	road.lateral_grip_assist = 0.05
	road.longitudinal_grip_ratio = 0.5
	errors += _save(road, PRESET_DIR + "surface_road.tres")

	var dirt: MVSurface = _load_or_new(PRESET_DIR + "surface_dirt.tres", "MVSurface")
	dirt.name = &"Dirt"
	dirt.tire_stiffness = 0.5
	dirt.coefficient_of_friction = 2.4
	dirt.rolling_resistance = 2.0
	dirt.lateral_grip_assist = 0.0
	dirt.longitudinal_grip_ratio = 0.5
	errors += _save(dirt, PRESET_DIR + "surface_dirt.tres")

	var grass: MVSurface = _load_or_new(PRESET_DIR + "surface_grass.tres", "MVSurface")
	grass.name = &"Grass"
	grass.tire_stiffness = 0.5
	grass.coefficient_of_friction = 2.0
	grass.rolling_resistance = 4.0
	grass.lateral_grip_assist = 0.0
	grass.longitudinal_grip_ratio = 0.5
	errors += _save(grass, PRESET_DIR + "surface_grass.tres")

	var library: MVSurfaceLibrary = _load_or_new(PRESET_DIR + "surface_library_default.tres", "MVSurfaceLibrary")
	library.surfaces = [road, dirt, grass]
	library.default_surface = road
	errors += _save(library, PRESET_DIR + "surface_library_default.tres")

	# Defaults of each class, saved as-is.
	errors += _save(_load_or_new(PRESET_DIR + "tire_default.tres", "MVTireSettings"), PRESET_DIR + "tire_default.tres")
	errors += _save(_load_or_new(PRESET_DIR + "suspension_default.tres", "MVSuspensionSettings"), PRESET_DIR + "suspension_default.tres")
	errors += _save(_load_or_new(PRESET_DIR + "brake_default.tres", "MVBrakeSettings"), PRESET_DIR + "brake_default.tres")

	# Values from addons/gevp/scenes/monster_truck.tscn (both axles are identical there).
	var truck_tire: MVTireSettings = _load_or_new(PRESET_DIR + "tire_monster_truck.tres", "MVTireSettings")
	truck_tire.tire_radius = 0.85
	truck_tire.tire_width = 1000.0
	truck_tire.wheel_mass = 363.0
	errors += _save(truck_tire, PRESET_DIR + "tire_monster_truck.tres")

	var truck_suspension: MVSuspensionSettings = _load_or_new(PRESET_DIR + "suspension_monster_truck.tres", "MVSuspensionSettings")
	truck_suspension.spring_length = 0.711
	truck_suspension.resting_ratio = 0.3
	truck_suspension.damping_ratio = 0.2
	truck_suspension.bump_damp_multiplier = 1.0
	truck_suspension.rebound_damp_multiplier = 1.0
	truck_suspension.anti_roll_ratio = 0.0
	errors += _save(truck_suspension, PRESET_DIR + "suspension_monster_truck.tres")

	if errors > 0:
		push_error("%d preset(s) failed to save" % errors)
	quit(1 if errors > 0 else 0)


# Reuses an existing preset so its UID survives regeneration; otherwise creates a fresh instance.
func _load_or_new(path: String, type: StringName) -> Resource:
	if ResourceLoader.exists(path):
		var existing := ResourceLoader.load(path, type, ResourceLoader.CACHE_MODE_IGNORE)
		if existing != null and existing.is_class(type):
			return existing
	return ClassDB.instantiate(type)


func _save(resource: Resource, path: String) -> int:
	resource.take_over_path(path)
	var err := ResourceSaver.save(resource, path)
	if err != OK:
		push_error("Could not save %s: %s" % [path, error_string(err)])
		return 1
	print("Saved ", path)
	return 0
