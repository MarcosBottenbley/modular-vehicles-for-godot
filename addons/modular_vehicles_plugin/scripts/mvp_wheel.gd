class_name MVPWheel
extends RayCast3D

## A brief description of the class's role and functionality if class_name is used.
##
## The description of the script, what it can do,
## and any further detail.

#region signals
#endregion

#region enums
#endregion

#region constants
#endregion

#region static variables
#endregion

#region export variables
## The [Node3D] correlating to this Wheel, which will have its
## rotation manipulated to make it spin and rotate.
## [br][br]
## [b]Tip:[/b] Make sure that your wheel is facing in the [b]+Z[/b] axis
## as this is considered the forward direction by both Godot and this script.
## [br][br]
## [b]Tip:[/b] If you're having issues with positioning your wheel,
## try parenting it to a [Node3D] and using that as the wheel node instead.
@export var wheel_node : Node3D

@export var surface_type := "Dirt"

#endregion

#region public variables
var beam_axle := 0.0
var is_powered := false

var ackermann := 0.15
var toe := 0.0

var moment_of_inertia := 0.0

var limit_spin := false
var spin := 0.0

var force_vector := Vector2.ZERO
var slip_vector := Vector2.ZERO
#endregion

#region private variables
var _axle : MVPAxle
var _opposite_wheel : MVPWheel

var _spring_current_length := 0.0
var _max_spring_length := 0.0
var _spring_force := 0.0
var _fast_damp_threshold := 127.0
var _damping_force := 0.0
var _previous_compression := 0.0

var _current_cof := 0.0
var _current_rolling_resistance := 0.0
var _current_lateral_grip_assist := 0.0
var _current_longitudinal_grip_ratio := 0.0
var _current_tire_stiffness := 0.0

var _abs_spin_difference_threshold := -12.0
var _spin_velocity_diff := 0.0
var _applied_torque := 0.0
var _local_velocity := Vector3.ZERO
var _previous_velocity := Vector3.ZERO
var _previous_global_position := Vector3.ZERO
var _antiroll_force := 0.0

var _last_collider : Object
var _last_collision_point := Vector3.ZERO
var _last_collision_normal := Vector3.ZERO
var _abs_enable_time := 0.0
#endregion

#region onready variables
@onready var mesh : MeshInstance3D = $MeshInstance3D
#endregion

#region static functions
#endregion

#region overridden built-in virtual functions

# Called when the object's script is instantiated, oftentimes after the object is initialized in memory
func _init() -> void:
	pass

# Called every time the node enters the scene tree.
func _enter_tree() -> void:
	pass

# Called when both the node and its children have entered the scene tree.
func _ready() -> void:
	pass

# Called when the node is about to leave the scene tree, after all its
# children received the _exit_tree() callback.
func _exit_tree() -> void:
	pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta : float) -> void:
	pass

# Called every physics frame.
func _physics_process(delta : float) -> void:
	if wheel_node:
		wheel_node.position.y = minf(0.0, -_spring_current_length)
		if not is_zero_approx(beam_axle):
			var wheel_lookat_vector := (_opposite_wheel.transform * _opposite_wheel.wheel_node.position) - (transform * wheel_node.position)
			wheel_node.rotation.z = wheel_lookat_vector.angle_to(Vector3.RIGHT * beam_axle) * signf(wheel_lookat_vector.y * beam_axle)
		wheel_node.rotation.x -= (wrapf(spin * delta, 0, TAU))

# Called once for every event before _unhandled_input(), allowing you to
# consume some events.
func _input(_event: InputEvent) -> void:
	pass

# Called once for every event.
func _unhandled_input(_event: InputEvent) -> void:
	pass

#endregion

#region overridden custom functions
#endregion

#region public functions
func initialize(is_on_left : bool, axle_is_powered : bool, opposite_wheel : MVPWheel) -> void:
	_axle = get_parent()
	_opposite_wheel = opposite_wheel
	wheel_node.rotation_order = EULER_ORDER_ZXY
	moment_of_inertia = 0.5 * _axle.wheel_mass * pow(_axle.tire_radius, 2)
	set_target_position(Vector3.DOWN * (_axle.spring_length + _axle.tire_radius))

	_abs_spin_difference_threshold = -absf(_axle.abs_spin_difference_threshold)
	_max_spring_length = _axle.spring_length
	_current_cof = _axle.coefficient_of_friction[surface_type]
	_current_rolling_resistance = _axle.rolling_resistance[surface_type]
	_current_lateral_grip_assist = _axle.lateral_grip_assist[surface_type]
	_current_longitudinal_grip_ratio = _axle.longitudinal_grip_ratio[surface_type]
	_current_tire_stiffness = 1000000.0 + 8000000.0 * _axle.tire_stiffnesses[surface_type]

	is_powered = axle_is_powered
	if is_on_left:
		mesh.set_rotation_degrees(Vector3(0, -180, 0))

func process_torque(drive : float, drive_inertia : float, brake_torque : float, allow_abs : bool, delta : float) -> float:
	var vehicle : MVPBody = _axle.get_body()
	## Add the torque the wheel produced last frame from surface friction
	var net_torque : float = force_vector.y * _axle.tire_radius
	var previous_spin := spin
	net_torque += drive

	## If antilock brakes are still active, don't apply brake torque
	if _abs_enable_time > vehicle.delta_time:
		brake_torque = 0.0
		allow_abs = false

	## If the wheel slip from braking is too great, enable the antilock brakes
	if absf(spin) > 5.0 and _spin_velocity_diff < _abs_spin_difference_threshold:
		if allow_abs and brake_torque > 0.0:
			brake_torque = 0.0
			_abs_enable_time = vehicle.delta_time + _axle.abs_pulse_time

	## Applied torque is used to ensure the wheels don't apply more force
	## than the motor or brakes applied to the wheel
	if is_zero_approx(spin):
		_applied_torque = absf(drive - brake_torque)
	else:
		_applied_torque = absf(drive - (brake_torque * signf(spin)))

	## If braking and nearly stopped, just stop the wheel completely.
	if absf(spin) < 5.0 and brake_torque > absf(net_torque):
		if allow_abs and absf(_local_velocity.z) > 2.0:
			_abs_enable_time = vehicle.delta_time + _axle.abs_pulse_time
		else:
			spin = 0.0
	else:
		## Spin the wheel based on the provided torque. The tire forces will handle
		## applying that force to the vehicle.
		net_torque -= brake_torque * signf(spin)
		var new_spin : float = spin + ((net_torque / (moment_of_inertia + drive_inertia)) * delta)
		if signf(spin) != signf(new_spin) and brake_torque > absf(drive):
			new_spin = 0.0
		spin = new_spin

	## The returned value is used to track wheel speed difference
	if is_zero_approx(drive * delta):
		return 0.5
	else:
		return (spin - previous_spin) * (moment_of_inertia + drive_inertia) / (drive * delta)

func process_forces(opposite_compression : float, braking : bool, delta : float) -> float:
	var vehicle : MVPBody = _axle.get_body()
	force_raycast_update()
	_previous_velocity = _local_velocity
	_local_velocity = (global_position - _previous_global_position) / delta * global_transform.basis
	_previous_global_position = global_position

	## Determine the surface the tire is on. Uses node groups
	if is_colliding():
		_last_collider = get_collider()
		_last_collision_point = get_collision_point()
		_last_collision_normal = get_collision_normal()
		var surface_groups : Array[StringName] = _last_collider.get_groups()
		if surface_groups.size() > 0:
			if surface_type != surface_groups[0]:
				surface_type = surface_groups[0]
				_current_cof = _axle.coefficient_of_friction[surface_type]
				_current_rolling_resistance = _axle.rolling_resistance[surface_type]
				_current_lateral_grip_assist = _axle.lateral_grip_assist[surface_type]
				_current_longitudinal_grip_ratio = _axle.longitudinal_grip_ratio[surface_type]
				_current_tire_stiffness = 1000000.0 + 8000000.0 * _axle.tire_stiffnesses[surface_type]
	else:
		_last_collider = null

	var compression := _process_suspension(opposite_compression, delta)

	if is_colliding() and _last_collider:
		_process_tires(braking, delta)
		var contact := _last_collision_point - vehicle.global_position
		if _spring_force > 0.0:
			vehicle.apply_force(_last_collision_normal * _spring_force, contact)
		else:
			## Apply a small amount of downward force if there is no spring force
			vehicle.apply_force(-global_transform.basis.y * vehicle.mass, global_position - vehicle.global_position)

		vehicle.apply_force(global_transform.basis.x * force_vector.x, contact)
		vehicle.apply_force(global_transform.basis.z * force_vector.y, contact)

		## Applies a torque on the vehicle body centered on the wheel. Gives the vehicle
		## more weight transfer when the center of gravity is really low.
		if braking:
			_axle.wheel_to_body_torque_multiplier = 1.0 / (_axle.braking_grip_multiplier + 1.0)
		vehicle.apply_force(-global_transform.basis.y * force_vector.y * 0.5 * _axle.wheel_to_body_torque_multiplier, to_global(Vector3.FORWARD * _axle.tire_radius))
		vehicle.apply_force(global_transform.basis.y * force_vector.y * 0.5 * _axle.wheel_to_body_torque_multiplier, to_global(Vector3.BACK * _axle.tire_radius))

		return compression

	else:
		force_vector = Vector2.ZERO
		slip_vector = Vector2.ZERO
		spin -= signf(spin) * delta * 2.0 / moment_of_inertia
		return 0.0

func steer(input : float, max_steering_angle : float) -> void:
	input *= _axle.steering_ratio
	rotation.y = (max_steering_angle * (input + (1 - cos(input * 0.5 * PI)) * ackermann)) + toe

func get_reaction_torque() -> float:
	return force_vector.y * _axle.tire_radius

func get_friction(normal_force : float, surface : String) -> float:
	var surface_cof := 1.0
	if _axle.coefficient_of_friction.has(surface):
		surface_cof = _axle.coefficient_of_friction[surface]
	return surface_cof * normal_force - (normal_force / (_axle.tire_width * _axle.contact_patch * 0.2))

func get_tire_radius() -> float:
	return _axle.tire_radius
#endregion

#region private functions
func _process_tires(braking : bool, delta : float) -> void:
	## This is a modified version of the brush tire model that removes the friction falloff beyond
	## the peak grip level.
	var local_planar := Vector2(_local_velocity.x, _local_velocity.z).normalized() * clampf(_local_velocity.length(), 0.0, 1.0)
	slip_vector.x = asin(clampf(-local_planar.x, -1.0, 1.0))
	slip_vector.y = 0.0

	var wheel_velocity := spin * _axle.tire_radius
	_spin_velocity_diff = wheel_velocity + _local_velocity.z
	var needed_rolling_force := ((_spin_velocity_diff * moment_of_inertia) / _axle.tire_radius) / delta
	var max_y_force := 0.0

	## Because the amount of force the tire applies is based on the amount of slip,
	## a maximum force is calculated based on the applied engine torque to prevent
	## the tire from creating too much force.
	if absf(_applied_torque) > absf(needed_rolling_force):
		max_y_force = absf(_applied_torque / _axle.tire_radius)
	else:
		max_y_force = absf(needed_rolling_force / _axle.tire_radius)

	var max_x_force := 0.0
	max_x_force = absf(_axle.get_mass_over_wheel() * _local_velocity.x) / delta

	var z_sign := signf(-_local_velocity.z)
	if _local_velocity.z == 0.0:
		z_sign = 1.0

	slip_vector.y = ((absf(_local_velocity.z) - (wheel_velocity * z_sign)) / (1.0 + absf(_local_velocity.z)))

	if slip_vector.is_zero_approx():
		slip_vector = Vector2(0.0001, 0.0001)

	var cornering_stiffness := 0.5 * _current_tire_stiffness * pow(_axle.contact_patch, 2.0)
	var friction := _current_cof * _spring_force - (_spring_force / (_axle.tire_width * _axle.contact_patch * 0.2))
	var deflect := 1.0 / (sqrt(pow(cornering_stiffness * slip_vector.y, 2.0) + pow(cornering_stiffness * slip_vector.x, 2.0)))

	## Adds in additional longitudinal grip when braking
	var braking_help := 1.0
	if slip_vector.y > 0.3 and braking:
		braking_help = (1 + (_axle.braking_grip_multiplier * clampf(absf(slip_vector.y), 0.0, 1.0)))

	var crit_length := friction * (1.0 - slip_vector.y) * _axle.contact_patch * (0.5 * deflect)
	if crit_length >= _axle.contact_patch:
		force_vector.y = cornering_stiffness * slip_vector.y / (1.0 - slip_vector.y)
		force_vector.x = cornering_stiffness * slip_vector.x / (1.0 - slip_vector.y)
	else:
		var brushx := (1.0 - friction * (1.0 - slip_vector.y) * (0.25 * deflect)) * deflect
		force_vector.y = friction * _current_longitudinal_grip_ratio * cornering_stiffness * slip_vector.y * brushx * braking_help * z_sign
		force_vector.x = friction * cornering_stiffness * slip_vector.x * brushx * (absf(slip_vector.x * _current_lateral_grip_assist) + 1.0)

	if absf(force_vector.y) > absf(max_y_force):
		force_vector.y = max_y_force * signf(force_vector.y)
		limit_spin = true
	else:
		limit_spin = false

	if absf(force_vector.x) > max_x_force:
		force_vector.x = max_x_force * signf(force_vector.x)

	force_vector.y -= _process_rolling_resistance() * signf(_local_velocity.z)

func _process_suspension(opposite_compression : float, delta : float) -> float:
	var vehicle : MVPBody = _axle.get_body()
	if is_colliding() and _last_collider:
		_spring_current_length = _last_collision_point.distance_to(global_position) - _axle.tire_radius
	else:
		_spring_current_length = _axle.spring_length

	var no_contact := false
	if _spring_current_length > _max_spring_length:
		_spring_current_length = _max_spring_length
		no_contact = true

	var bottom_out := false
	if _spring_current_length < 0.0:
		_spring_current_length = 0.0
		bottom_out = true

	var compression := (_axle.spring_length - _spring_current_length) * 1000.0

	var spring_speed_mm_per_seconds := (compression - _previous_compression) / delta
	_previous_compression = compression

	_spring_force = compression * _axle.spring_rate
	_antiroll_force = _axle.get_antiroll() * (compression - opposite_compression)
	_spring_force += _antiroll_force

	## If the suspension is bottomed out, apply some additional forces to help keep the vehicle body
	## from colliding with the surface.
	var bottom_out_damping := 0.0
	var bottom_out_damping_fast := 0.0
	var bottom_out_force := 0.0
	if bottom_out:
		var gravity_on_spring := clampf(global_transform.basis.y.dot(-vehicle.current_gravity.normalized()), 0.0, 1.0)
		bottom_out_force = (((_axle.get_mass_over_wheel() * clampf(spring_speed_mm_per_seconds * 0.001, 0.0, 5.0)) / delta) + (_axle.get_mass_over_wheel() * vehicle.current_gravity.length() * gravity_on_spring)) * _axle.bump_stop_multiplier
		bottom_out_damping = -_axle.get_slow_bump()
		bottom_out_damping_fast = -_axle.get_fast_bump()

	if spring_speed_mm_per_seconds >= 0:
		if spring_speed_mm_per_seconds > _fast_damp_threshold:
			_damping_force = ((spring_speed_mm_per_seconds - _fast_damp_threshold) * (_axle.get_fast_bump() + bottom_out_damping_fast)) + (_fast_damp_threshold * (_axle.get_slow_bump() + bottom_out_damping))
		else:
			_damping_force = spring_speed_mm_per_seconds * (_axle.get_slow_bump() + bottom_out_damping)
	else :
		if spring_speed_mm_per_seconds < -_fast_damp_threshold:
			_damping_force = ((spring_speed_mm_per_seconds + _fast_damp_threshold) * _axle.get_fast_rebound()) + (-_fast_damp_threshold * _axle.get_slow_rebound())
		else:
			_damping_force = spring_speed_mm_per_seconds * _axle.get_slow_rebound()

	_spring_force += _damping_force

	_spring_force = maxf(0, _spring_force + bottom_out_force)

	_max_spring_length = clampf((((_spring_force / _axle.wheel_mass) - spring_speed_mm_per_seconds) * delta * 0.001) + _spring_current_length, 0.0, _axle.spring_length)

	if no_contact:
		_spring_force = 0.0

	return compression

func _process_rolling_resistance() -> float:
	var rolling_resistance_coefficient := 0.005 + (0.5 * (0.01 + (0.0095 * pow(_local_velocity.z * 0.036, 2))))
	return rolling_resistance_coefficient * _spring_force * _current_rolling_resistance
#endregion

#region subclasses
#endregion
