# Headless smoke test for the settings resources (card B05).
#
# Run from the repository root after building the extension and importing the project once:
#   godot --headless --path . --import
#   godot --headless --path . -s tests/scripts/check_settings.gd
#
# Exits with code 1 when any check fails. Card B04 will wrap these checks in GUT; until then
# this script is the automated verification for the resource classes and presets.
extends SceneTree

const PRESET_DIR := "res://addons/modular_vehicles/presets/"
const EPSILON := 0.00001

# class name -> { property: expected default }. Must match the GDScript defaults in
# addons/gevp/scripts/vehicle.gd and wheel.gd (see docs/settings.md).
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

# One property per class that is changed to verify the `changed` signal and clamping.
const CHANGE_PROBE := {
	"MVSurface": ["tire_stiffness", 2.5],
	"MVSurfaceLibrary": ["default_surface", null], # replaced by a fresh MVSurface at runtime
	"MVTireSettings": ["tire_radius", 0.5],
	"MVSuspensionSettings": ["spring_length", 0.3],
	"MVBrakeSettings": ["abs_pulse_time", 0.1],
}

# Expected class of every preset file.
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

var _failures := 0
var _checks := 0
var _changed_count := 0


func _init() -> void:
	_check_classes()
	_check_presets()
	_check_surface_library()
	_check_round_trip()
	_check_clamping()
	print("check_settings: %d checks, %d failures" % [_checks, _failures])
	quit(1 if _failures > 0 else 0)


func _expect(condition: bool, message: String) -> void:
	_checks += 1
	if not condition:
		_failures += 1
		push_error("FAIL: " + message)
		printerr("FAIL: " + message)


func _values_equal(a: Variant, b: Variant) -> bool:
	if typeof(a) == TYPE_FLOAT or typeof(b) == TYPE_FLOAT:
		return is_equal_approx(float(a), float(b)) or absf(float(a) - float(b)) < EPSILON
	return a == b


func _on_changed() -> void:
	_changed_count += 1


func _check_classes() -> void:
	for class_name_ in DEFAULTS:
		_expect(ClassDB.class_exists(class_name_), "%s is registered in ClassDB" % class_name_)
		if not ClassDB.class_exists(class_name_):
			continue
		var instance: Resource = ClassDB.instantiate(class_name_)
		_expect(instance != null, "%s can be instantiated" % class_name_)
		if instance == null:
			continue
		_expect(instance.is_class("Resource"), "%s is a Resource" % class_name_)

		var defaults: Dictionary = DEFAULTS[class_name_]
		for property in defaults:
			var actual: Variant = instance.get(property)
			_expect(_values_equal(actual, defaults[property]),
					"%s.%s default is %s (got %s)" % [class_name_, property, defaults[property], actual])

		# Setting a property must emit `changed` exactly once; setting the same value again must not.
		var probe: Array = CHANGE_PROBE[class_name_]
		var property: String = probe[0]
		var value: Variant = probe[1]
		if class_name_ == "MVSurfaceLibrary":
			value = MVSurface.new()
		_changed_count = 0
		instance.changed.connect(_on_changed)
		instance.set(property, value)
		_expect(_changed_count == 1, "%s emits changed when %s is set (count %d)" % [class_name_, property, _changed_count])
		_expect(_values_equal(instance.get(property), value), "%s.%s stores the new value" % [class_name_, property])
		instance.set(property, value)
		_expect(_changed_count == 1, "%s does not emit changed for an unchanged %s" % [class_name_, property])
		instance.changed.disconnect(_on_changed)


func _check_presets() -> void:
	var dir := DirAccess.open(PRESET_DIR)
	_expect(dir != null, "preset folder %s exists" % PRESET_DIR)
	if dir == null:
		return
	var found: Array[String] = []
	for file in dir.get_files():
		if file.ends_with(".tres"):
			found.append(file)
	for file in PRESETS:
		_expect(found.has(file), "preset %s is present" % file)
	for file in found:
		var path := PRESET_DIR + file
		var resource := ResourceLoader.load(path, "", ResourceLoader.CACHE_MODE_IGNORE)
		_expect(resource != null, "preset %s loads" % file)
		if resource == null:
			continue
		var expected: String = PRESETS.get(file, "")
		_expect(expected != "", "preset %s is listed in check_settings.gd" % file)
		if expected != "":
			_expect(resource.get_class() == expected, "preset %s is a %s (got %s)" % [file, expected, resource.get_class()])

	# Spot-check a few values against the GDScript sources.
	var road: MVSurface = load(PRESET_DIR + "surface_road.tres")
	if road:
		_expect(road.name == &"Road", "surface_road.tres is named Road")
		_expect(_values_equal(road.coefficient_of_friction, 3.0), "surface_road.tres friction is 3.0")
	var truck: MVTireSettings = load(PRESET_DIR + "tire_monster_truck.tres")
	if truck:
		_expect(_values_equal(truck.tire_radius, 0.85), "tire_monster_truck.tres radius is 0.85")
		_expect(_values_equal(truck.wheel_mass, 363.0), "tire_monster_truck.tres wheel mass is 363")
	var truck_suspension: MVSuspensionSettings = load(PRESET_DIR + "suspension_monster_truck.tres")
	if truck_suspension:
		_expect(_values_equal(truck_suspension.spring_length, 0.711), "suspension_monster_truck.tres spring length is 0.711")
		_expect(_values_equal(truck_suspension.anti_roll_ratio, 0.0), "suspension_monster_truck.tres has no anti-roll bar")


func _check_surface_library() -> void:
	var library: MVSurfaceLibrary = load(PRESET_DIR + "surface_library_default.tres")
	_expect(library != null, "surface_library_default.tres loads")
	if library == null:
		return
	_expect(library.surfaces.size() == 3, "default library holds 3 surfaces (got %d)" % library.surfaces.size())
	_expect(library.default_surface != null and library.default_surface.name == &"Road", "default library falls back to Road")

	var dirt := library.find_surface(&"Dirt")
	_expect(dirt != null and dirt.name == &"Dirt", "find_surface(\"Dirt\") returns the Dirt surface")
	var unknown := library.find_surface(&"Lava")
	_expect(unknown != null and unknown == library.default_surface, "find_surface(\"Lava\") returns the default surface")

	# An empty library still answers with the built-in Road surface.
	var empty := MVSurfaceLibrary.new()
	var builtin := empty.find_surface(&"Anything")
	_expect(builtin != null and builtin.name == &"Road", "empty library returns the built-in Road surface")
	_expect(builtin == empty.find_surface(&"Other"), "built-in Road surface is shared")


func _check_round_trip() -> void:
	# Load, change a value, save to a temporary path, reload and compare.
	var temp_dir := "user://check_settings_tmp/"
	DirAccess.make_dir_recursive_absolute(temp_dir)
	for file in PRESETS:
		var source: String = PRESET_DIR + file
		var resource := ResourceLoader.load(source, "", ResourceLoader.CACHE_MODE_IGNORE)
		if resource == null:
			continue
		var property := _first_float_property(resource)
		var new_value := 0.0
		if property != "":
			new_value = float(resource.get(property)) + 0.25
			resource.set(property, new_value)
		var temp_path: String = temp_dir + file
		var err := ResourceSaver.save(resource, temp_path)
		_expect(err == OK, "%s saves to %s (%s)" % [file, temp_path, error_string(err)])
		if err != OK:
			continue
		var reloaded := ResourceLoader.load(temp_path, "", ResourceLoader.CACHE_MODE_IGNORE)
		_expect(reloaded != null and reloaded.get_class() == resource.get_class(), "%s reloads as %s" % [file, resource.get_class()])
		if reloaded != null and property != "":
			_expect(_values_equal(reloaded.get(property), new_value), "%s keeps %s = %s after round trip" % [file, property, new_value])
		DirAccess.remove_absolute(temp_path)
	DirAccess.remove_absolute(temp_dir)


func _first_float_property(resource: Resource) -> String:
	for info in resource.get_property_list():
		if info.type == TYPE_FLOAT and info.usage & PROPERTY_USAGE_SCRIPT_VARIABLE == 0 and info.usage & PROPERTY_USAGE_EDITOR:
			return info.name
	return ""


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
