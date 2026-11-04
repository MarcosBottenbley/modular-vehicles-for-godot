extends SceneTree
## Headless smoke test for the settings resources (card B05).
##
## Run from the repository root after building the extension and importing the project once:
##   godot --headless --path . --import
##   godot --headless --path . -s tests/scripts/check_settings.gd
##
## Exits with code 1 when any check fails. Card B04 will wrap these checks in GUT; until then
## this script is the automated verification for the resource classes and presets.
##
## Terms:
##   settings resource: an MVSurface, MVSurfaceLibrary, MVTireSettings, MVSuspensionSettings
##     or MVBrakeSettings instance.
##   default: the value a freshly instantiated settings resource holds. Must match the
##     GDScript defaults of addons/gevp/scripts/vehicle.gd and wheel.gd (see docs/settings.md).
##   change probe: one property per class that the test sets to prove the `changed` signal
##     fires once per real change and not for an unchanged value.
##   preset: a .tres file in addons/modular_vehicles/presets/ holding one settings resource.
##   round trip: load a preset, change a value, save it under user://, reload it and compare.
##   check: one boolean assertion; failures are counted and printed, never fatal mid-run.

const PRESET_DIR := "res://addons/modular_vehicles/presets/"
const TEMP_DIR := "user://check_settings_tmp/"
## Tolerance for float comparisons where is_equal_approx is too strict for stored floats.
const EPSILON := 0.00001
## Offset added to a float property during the round trip so the reloaded value is distinct.
const ROUND_TRIP_OFFSET := 0.25

## Class name -> { property: default }.
const DEFAULTS := {
	"MVSurface": {
		"name": &"Road",
		"tire_stiffness": 10.0,
		"coefficient_of_friction": 3.0,
		"rolling_resistance": 1.0,
		"lateral_grip_assist": 0.05,
		"longitudinal_grip_ratio": 0.5,
	},
	"MVSurfaceLibrary": {
		"surfaces": [],
		"default_surface": null,
	},
	"MVTireSettings": {
		"tire_radius": 0.3,
		"tire_width": 245.0,
		"wheel_mass": 15.0,
		"contact_patch": 0.2,
		"braking_grip_multiplier": 1.5,
		"wheel_to_body_torque_multiplier": 1.0,
		"grip_multiplier": 1.0,
		"stiffness_multiplier": 1.0,
		"rolling_resistance_multiplier": 1.0,
	},
	"MVSuspensionSettings": {
		"spring_length": 0.15,
		"resting_ratio": 0.5,
		"spring_rate_mode": 0,
		"manual_spring_rate": 49000.0,
		"damping_ratio": 0.4,
		"bump_damp_multiplier": 0.6667,
		"rebound_damp_multiplier": 1.5,
		"fast_damp_threshold": 0.127,
		"manual_damping": 3400.0,
		"bump_stop_multiplier": 1.0,
		"anti_roll_ratio": 0.25,
		"camber": 0.01745329,
		"toe": 0.01,
	},
	"MVBrakeSettings": {
		"abs_enabled": true,
		"abs_pulse_time": 0.03,
		"abs_spin_difference_threshold": 12.0,
		"bias_weight": -1.0,
	},
}

## Class name -> [property, new value]. The library's value is a fresh MVSurface at runtime.
const CHANGE_PROBES := {
	"MVSurface": ["tire_stiffness", 2.5],
	"MVSurfaceLibrary": ["default_surface", null],
	"MVTireSettings": ["tire_radius", 0.5],
	"MVSuspensionSettings": ["spring_length", 0.3],
	"MVBrakeSettings": ["abs_pulse_time", 0.1],
}

## Preset file name -> class it must hold.
const PRESETS := {
	"surface_road.tres": "MVSurface",
	"surface_dirt.tres": "MVSurface",
	"surface_grass.tres": "MVSurface",
	"surface_library_default.tres": "MVSurfaceLibrary",
	"tire_default.tres": "MVTireSettings",
	"tire_monster_truck.tres": "MVTireSettings",
	"suspension_default.tres": "MVSuspensionSettings",
	"suspension_monster_truck.tres": "MVSuspensionSettings",
	"brake_default.tres": "MVBrakeSettings",
}

var _failure_count: int = 0
var _check_count: int = 0
var _changed_signal_count: int = 0


## Entry point: runs every check group, prints the totals and quits with 1 on any failure.
## Returns: nothing. Calls quit() with the exit code the shell sees.
func _init() -> void:
	_check_classes()
	_check_presets()
	_check_surface_library()
	_check_round_trip()
	_check_clamping()
	print("check_settings: %d checks, %d failures" % [_check_count, _failure_count])
	quit(1 if _failure_count > 0 else 0)


## Records one check.
## condition: the assertion; false counts as a failure.
## message: what was asserted, printed on failure.
## Returns: nothing. Increments the counters that _init() reports.
func _expect(condition: bool, message: String) -> void:
	_check_count += 1
	if not condition:
		_failure_count += 1
		push_error("FAIL: " + message)
		printerr("FAIL: " + message)


## Compares two values, with a tolerance when either is a float.
## first: a value read from a resource.
## second: the expected value.
## Returns: true when they are equal, or within EPSILON for floats.
func _values_equal(first: Variant, second: Variant) -> bool:
	if typeof(first) == TYPE_FLOAT or typeof(second) == TYPE_FLOAT:
		var difference: float = absf(float(first) - float(second))
		return is_equal_approx(float(first), float(second)) or difference < EPSILON
	return first == second


## Signal target for the change probe.
## Returns: nothing. Counts how often `changed` fired since the counter was reset.
func _on_changed() -> void:
	_changed_signal_count += 1


## Checks that every settings class is registered, instantiates, holds its defaults and
## emits `changed` correctly for its change probe.
## Returns: nothing. Records checks through _expect().
func _check_classes() -> void:
	for class_name_: String in DEFAULTS:
		_expect(ClassDB.class_exists(class_name_), "%s is registered in ClassDB" % class_name_)
		if not ClassDB.class_exists(class_name_):
			continue
		var instance: Resource = ClassDB.instantiate(class_name_)
		_expect(instance != null, "%s can be instantiated" % class_name_)
		if instance == null:
			continue
		_expect(instance.is_class("Resource"), "%s is a Resource" % class_name_)
		_check_defaults(instance, class_name_, DEFAULTS[class_name_])
		_check_change_probe(instance, class_name_)


## Compares every default of one instance against the expected table.
## instance: a freshly created settings resource.
## class_name_: its class, for the messages.
## defaults: property -> expected default.
## Returns: nothing. Records one check per property.
func _check_defaults(instance: Resource, class_name_: String, defaults: Dictionary) -> void:
	for property: String in defaults:
		var actual: Variant = instance.get(property)
		var expected: Variant = defaults[property]
		_expect(
			_values_equal(actual, expected),
			"%s.%s default is %s (got %s)" % [class_name_, property, expected, actual],
		)


## Sets the change probe of one instance twice: the first set must emit `changed` exactly
## once and store the value, the second (same value) must not emit again.
## instance: a settings resource to probe.
## class_name_: its class, selects the probe and labels the messages.
## Returns: nothing. Records three checks.
func _check_change_probe(instance: Resource, class_name_: String) -> void:
	var probe: Array = CHANGE_PROBES[class_name_]
	var property: String = probe[0]
	var value: Variant = probe[1]
	if class_name_ == "MVSurfaceLibrary":
		value = MVSurface.new()

	var label: String = "%s.%s" % [class_name_, property]
	_changed_signal_count = 0
	instance.changed.connect(_on_changed)
	instance.set(property, value)
	_expect(
		_changed_signal_count == 1,
		"%s emits changed when set (count %d)" % [label, _changed_signal_count],
	)
	_expect(_values_equal(instance.get(property), value), "%s stores the new value" % label)
	instance.set(property, value)
	_expect(_changed_signal_count == 1, "%s does not emit changed when unchanged" % label)
	instance.changed.disconnect(_on_changed)


## Checks that every expected preset exists, that every preset in the folder loads as the
## listed class, and spot-checks values against the GDScript sources.
## Returns: nothing. Records checks through _expect().
func _check_presets() -> void:
	var found: Array[String] = _list_preset_files()
	for file_name: String in PRESETS:
		_expect(found.has(file_name), "preset %s is present" % file_name)
	for file_name: String in found:
		_check_preset_type(file_name)
	_check_preset_values()


## Lists the .tres files in the preset folder.
## Returns: file names without the folder; empty when the folder is missing (a check fails).
func _list_preset_files() -> Array[String]:
	var found: Array[String] = []
	var directory: DirAccess = DirAccess.open(PRESET_DIR)
	_expect(directory != null, "preset folder %s exists" % PRESET_DIR)
	if directory == null:
		return found
	for file_name: String in directory.get_files():
		if file_name.ends_with(".tres"):
			found.append(file_name)
	return found


## Loads one preset and checks it is listed and of the listed class.
## file_name: preset file name inside PRESET_DIR.
## Returns: nothing. Records up to three checks.
func _check_preset_type(file_name: String) -> void:
	var resource: Resource = _load_preset_uncached(file_name)
	_expect(resource != null, "preset %s loads" % file_name)
	if resource == null:
		return
	var expected: String = PRESETS.get(file_name, "")
	_expect(expected != "", "preset %s is listed in check_settings.gd" % file_name)
	if expected != "":
		_expect(
			resource.get_class() == expected,
			"preset %s is a %s (got %s)" % [file_name, expected, resource.get_class()],
		)


## Spot-checks a few preset values against vehicle.gd and monster_truck.tscn.
## Returns: nothing. Records checks through _expect().
func _check_preset_values() -> void:
	var road: MVSurface = load(PRESET_DIR + "surface_road.tres")
	if road:
		_expect(road.name == &"Road", "surface_road.tres is named Road")
		_expect(_values_equal(road.coefficient_of_friction, 3.0), "surface_road.tres friction is 3.0")
	var truck_tire: MVTireSettings = load(PRESET_DIR + "tire_monster_truck.tres")
	if truck_tire:
		_expect(_values_equal(truck_tire.tire_radius, 0.85), "tire_monster_truck.tres radius is 0.85")
		_expect(_values_equal(truck_tire.wheel_mass, 363.0), "tire_monster_truck.tres wheel mass is 363")
	var truck_suspension: MVSuspensionSettings = load(PRESET_DIR + "suspension_monster_truck.tres")
	if truck_suspension:
		_expect(
			_values_equal(truck_suspension.spring_length, 0.711),
			"suspension_monster_truck.tres spring length is 0.711",
		)
		_expect(
			_values_equal(truck_suspension.anti_roll_ratio, 0.0),
			"suspension_monster_truck.tres has no anti-roll bar",
		)


## Checks find_surface() on the default library (known and unknown names) and on an empty
## library (built-in road fallback, shared between calls).
## Returns: nothing. Records checks through _expect().
func _check_surface_library() -> void:
	var library: MVSurfaceLibrary = load(PRESET_DIR + "surface_library_default.tres")
	_expect(library != null, "surface_library_default.tres loads")
	if library == null:
		return
	var surface_count: int = library.surfaces.size()
	_expect(surface_count == 3, "default library holds 3 surfaces (got %d)" % surface_count)
	_expect(
		library.default_surface != null and library.default_surface.name == &"Road",
		"default library falls back to Road",
	)

	var dirt: MVSurface = library.find_surface(&"Dirt")
	_expect(dirt != null and dirt.name == &"Dirt", "find_surface(\"Dirt\") returns the Dirt surface")
	var unknown: MVSurface = library.find_surface(&"Lava")
	_expect(
		unknown != null and unknown == library.default_surface,
		"find_surface(\"Lava\") returns the default surface",
	)

	var empty_library := MVSurfaceLibrary.new()
	var builtin: MVSurface = empty_library.find_surface(&"Anything")
	_expect(builtin != null and builtin.name == &"Road", "empty library returns the built-in Road")
	_expect(builtin == empty_library.find_surface(&"Other"), "built-in Road surface is shared")


## Round-trips every preset through a temporary folder under user://.
## Returns: nothing. Records checks through _expect() and removes the temporary files.
func _check_round_trip() -> void:
	DirAccess.make_dir_recursive_absolute(TEMP_DIR)
	for file_name: String in PRESETS:
		_round_trip_preset(file_name)
	DirAccess.remove_absolute(TEMP_DIR)


## Round-trips one preset: changes its first editable float property by ROUND_TRIP_OFFSET,
## saves to TEMP_DIR, reloads and compares class and value.
## file_name: preset file name inside PRESET_DIR.
## Returns: nothing. Records up to three checks and deletes the temporary file.
func _round_trip_preset(file_name: String) -> void:
	var resource: Resource = _load_preset_uncached(file_name)
	if resource == null:
		return
	var property: String = _first_float_property(resource)
	var new_value: float = 0.0
	if property != "":
		new_value = float(resource.get(property)) + ROUND_TRIP_OFFSET
		resource.set(property, new_value)

	var temp_path: String = TEMP_DIR + file_name
	var error: Error = ResourceSaver.save(resource, temp_path)
	_expect(error == OK, "%s saves to %s (%s)" % [file_name, temp_path, error_string(error)])
	if error != OK:
		return
	var reloaded: Resource = ResourceLoader.load(temp_path, "", ResourceLoader.CACHE_MODE_IGNORE)
	_expect(
		reloaded != null and reloaded.get_class() == resource.get_class(),
		"%s reloads as %s" % [file_name, resource.get_class()],
	)
	if reloaded != null and property != "":
		_expect(
			_values_equal(reloaded.get(property), new_value),
			"%s keeps %s = %s after round trip" % [file_name, property, new_value],
		)
	DirAccess.remove_absolute(temp_path)


## Loads a preset bypassing the resource cache, so edits made by one check never leak into
## the next.
## file_name: preset file name inside PRESET_DIR.
## Returns: the loaded settings resource, or null when loading failed.
func _load_preset_uncached(file_name: String) -> Resource:
	return ResourceLoader.load(PRESET_DIR + file_name, "", ResourceLoader.CACHE_MODE_IGNORE)


## Finds a float property the round trip can change.
## resource: the settings resource to inspect.
## Returns: the first editor-visible float property name, or "" when the class has none
## (MVSurfaceLibrary), in which case only the class is compared.
func _first_float_property(resource: Resource) -> String:
	for info: Dictionary in resource.get_property_list():
		var is_script_variable: bool = info.usage & PROPERTY_USAGE_SCRIPT_VARIABLE != 0
		var is_editor_visible: bool = info.usage & PROPERTY_USAGE_EDITOR != 0
		if info.type == TYPE_FLOAT and not is_script_variable and is_editor_visible:
			return info.name
	return ""


## Checks the clamps and the enum setter that the setters apply.
## Returns: nothing. Records checks through _expect().
func _check_clamping() -> void:
	var suspension := MVSuspensionSettings.new()
	suspension.resting_ratio = 0.0
	_expect(_values_equal(suspension.resting_ratio, 0.01), "resting_ratio clamps to 0.01")
	suspension.resting_ratio = 5.0
	_expect(_values_equal(suspension.resting_ratio, 1.0), "resting_ratio clamps to 1.0")
	suspension.spring_rate_mode = MVSuspensionSettings.SPRING_RATE_MANUAL
	_expect(suspension.spring_rate_mode == 1, "spring_rate_mode accepts SPRING_RATE_MANUAL")

	var tire := MVTireSettings.new()
	tire.tire_radius = -1.0
	_expect(tire.tire_radius > 0.0, "tire_radius cannot be negative")

	var brake := MVBrakeSettings.new()
	brake.bias_weight = -5.0
	_expect(_values_equal(brake.bias_weight, -1.0), "bias_weight clamps to -1")
