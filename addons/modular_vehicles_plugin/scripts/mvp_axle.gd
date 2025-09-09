class_name MVPAxle
extends Node3D

## This node models a Axle on the vehicle. It acts as the bridge between the vehicle and the wheels.
## An Axle can store 1 or 2 wheels. Since an axle can have more than one wheel there are several wheel
## parameters here so that the wheels don't end up with two different values if attached to the same axle.
## This node is designed with the intention of both wheels attached to the axle matching exactly all the time.

#region signals
#endregion

#region enums
#endregion

#region constants
#endregion

#region static variables
#endregion

#region export variables
@export var is_powered := false

@export var handbrake := false

## If true the wheels of this axle will be aligned as if they were attached to
## a beam axle. This setting does not affect vehicle handling.
@export var is_beam_axle := false

@export var is_front_axle := false

## The ratio that the wheels turn based on steering input.
## [br]The higher this value, the more the wheels will turn due to steering input.
@export_range(0.0, 1.0) var steering_ratio := 0.0

@export_group('Wheels')

## Number of wheels attached to the axle
## An axle only supports 1 or 2 wheels
@export_range(1,2) var wheel_num : int = 2

## Wheel mass in kilograms.
@export var wheel_mass := 15.0

@export_group('Drivetrain')
## The amount of torque vectoring to apply to the axle based on steering input.
## Only functions if the differential is locked.
## A value of 1.0 would apply all torque to the outside wheel.
@export_range(0.0, 1.0) var torque_vectoring := 0.0

## The wheels of the axle will be forced to spin the same speed if there
## is at least this much torque applied. Keeps vehicle from spinning one wheel.
## Torque is measured after multiplied by the current gear ratio.
## Negative values will disable.
@export var locking_differential_engage_torque := 200.0

@export_group('Suspension')
## The amount of suspension travel in meters. Rear suspension typically has
## more travel than the front.
@export var spring_length := 0.2

## How much the spring is compressed when the vehicle is at rest.
## This is used to calculate the approriate spring rate for the wheel.
## A value of 1 would be a fully compressed spring. With a value of 0.5 the
## suspension will rest at the center of it's length.
@export_range(0.0, 1.0) var resting_ratio := 0.5

## Damping ratio is used to calculate the damping forces on the spring.
## A value of 1 would be critically damped. Passenger cars typically have a
## ratio around 0.3, while a race car could be as high as 0.9.
@export_range(0.0, 1.0) var damping_ratio := 0.4

## Bump damping multiplier applied to the damping force calulated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var bump_damp_multiplier := 0.6667

## Rebound damping multiplier applied to the damping force calulated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var rebound_damp_multiplier := 1.5

## Antiroll bar stiffness as a ratio to spring stiffness.
@export var arb_ratio := 0.25

## Wheel camber isn't simulated, but giving the raycast a slight angle helps
## with simulation stability. Measured in radians.
@export var camber := 0.01745329

## Toe of the tires measured in radians.
@export var toe := 0.01

## Multiplier for the force applied when the suspension is fully compressed.
## If the vehicle bounces off large bumps, reducing this will help.
@export var bump_stop_multiplier := 1.0

@export var compression : float = 0.6

@export_group('Brakes')
## Ratio of total brake force applied to this axle.
## Values of all axles should add to 1.
@export_range(0.0, 1.0, 0.1) var brake_bias : float = 0.5

## Multiplies the automatically calculated brake force.
@export var brake_force_multiplier := 1.0

## How long the ABS releases the brake, in seconds, when the
## spin threshold is crossed.
@export var abs_pulse_time := 0.3

## The difference in speed required between the wheel and the
## driving surface for ABS to engage.
@export var abs_spin_difference_threshold := 12.0

@export_group("Tires")
## Represents the length of the tire contact patch in the brush tire model.
@export var contact_patch := 0.2

## Provides additional longitudinal grip when braking.
@export var braking_grip_multiplier := 1.5

## Tire force applied to the ground is also applied to the vehicle body as a
## torque centered on the wheel.
@export var wheel_to_body_torque_multiplier := 1.0

## Represents tire stiffness in the brush tire model. Higher values increase
## the responsivness of the tire.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var tire_stiffnesses := { "Road" : 10.0, "Dirt" : 0.5, "Grass" : 0.5 }

## A multiplier for the amount of force a tire can apply based on the surface.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var coefficient_of_friction := { "Road" : 3.0, "Dirt" : 2.4, "Grass" : 2.0 }

## A multiplier for the amount of rolling resistance force based on the surface.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var rolling_resistance := { "Road" : 1.0, "Dirt" : 2.0, "Grass" : 4.0 }

## A multiplier to provide more grip based on the amount of lateral wheel slip.
## This can be used to keep vehicles from sliding a long distance, but may provide
## unrealistically high amounts of grip.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var lateral_grip_assist := { "Road" : 0.05, "Dirt" : 0.0, "Grass" : 0.0}

## A multiplier to adjust longitudinal grip to differ from lateral grip.
## Useful for allowing vehicles to have wheel spin and maintain high lateral grip.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var longitudinal_grip_ratio := { "Road" : 0.5, "Dirt": 0.5, "Grass" : 0.5}

## Tire radius in meters
@export var tire_radius := 0.3

## Tire width in millimeters. The width doesn't directly affect tire friction,
## but reduces the effects of tire load sensitivity.
@export var tire_width := 205.0
#endregion

#region public variables
## Assign this to the Wheel [RayCast3D] that is this axles left wheel.
var left_wheel : MVPWheel

## Assign this to the Wheel [RayCast3D] that is this axles right wheel.
var right_wheel : MVPWheel

var tire_size_correction := 0.0
var inertia := 0.0

var rotation_split := 0.5
var applied_split := 0.5

var track_width : float
var mass_over_axle : float
var is_braking := false
var brake_force := 0.0
var max_brake_force := 0.0
var handbrake_force := 0.0
var max_handbrake_force := 0.0

var spring_rate := 0.0
#endregion

#region private variables
var _vehicle : MVPBody
var _suspension_compression_left := 0.0
var _suspension_compression_right := 0.0

var _spring_force := 0.0
var _damping_rate := 0.0
#endregion

#region onready variables
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
	## Spring compression values are kept for antiroll bar calculations
	var previous_compression_left : float = _suspension_compression_left
	_suspension_compression_left = left_wheel.process_forces(_suspension_compression_right, is_braking, delta)
	_suspension_compression_right = right_wheel.process_forces(previous_compression_left, is_braking, delta)

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
func initialize() -> void:
	_vehicle = get_parent()
	left_wheel.initialize(true, is_powered, right_wheel)
	right_wheel.initialize(false, is_powered, left_wheel)
	position = left_wheel.position.lerp(right_wheel.position, 0.5)
	_spring_force = spring_length * compression * 1000.0 * spring_rate * 2.0

	if is_beam_axle:
		left_wheel.beam_axle = 1.0
		right_wheel.beam_axle = -1.0

	track_width = right_wheel.position.x - left_wheel.position.x

	left_wheel.rotation.z = -camber
	right_wheel.rotation.z = camber

	left_wheel.toe = -toe
	right_wheel.toe = toe

	inertia += left_wheel.moment_of_inertia + right_wheel.moment_of_inertia

	if is_powered:
		_vehicle.drive_axles_inertia += left_wheel.moment_of_inertia + right_wheel.moment_of_inertia
		_vehicle._powered_wheels.append(left_wheel)
		_vehicle._powered_wheels.append(right_wheel)
		#_vehicle.average_drive_wheel_radius += left_wheel.tire_radius + right_wheel.tire_radius

func get_body() -> MVPBody:
	return _vehicle

func get_spin() -> float:
	var spin := maxf(left_wheel.spin, right_wheel.spin)
	return spin * tire_size_correction

func get_average_spin() -> float:
	var spin : float = left_wheel.spin + right_wheel.spin
	return spin / 2

func get_max_wheel_slip_y() -> float:
	var slip := maxf(left_wheel.slip_vector.y, right_wheel.slip_vector.y)
	return slip

func get_max_slip_angle() -> float:
	return maxf(left_wheel.slip_vector.x, right_wheel.slip_vector.x)

func get_antiroll() -> float:
	return spring_rate * arb_ratio

func get_slow_bump() -> float:
	return _damping_rate * bump_damp_multiplier

func get_slow_rebound() -> float:
	return _damping_rate * rebound_damp_multiplier

func get_fast_bump() -> float:
	return (_damping_rate * rebound_damp_multiplier) / wheel_num

func get_fast_rebound() -> float:
	return (_damping_rate * rebound_damp_multiplier) / wheel_num

func get_mass_over_wheel() -> float:
	return mass_over_axle / wheel_num

func get_abs_spin_difference_threshold() -> float:
	return -absf(abs_spin_difference_threshold)

func set_ackermann(wheel_base : float, max_steering_angle : float) -> void:
	var ackermann := (atan((wheel_base * tan(max_steering_angle)) / (wheel_base - (track_width * 0.5 * tan(max_steering_angle)))) / max_steering_angle) - 1.0
	left_wheel.ackermann = ackermann
	right_wheel.ackermann = -ackermann

func set_spring_rate(weight : float) -> void:
	var corrected_resting_ratio := (spring_length * resting_ratio) / spring_length
	var target_compression := spring_length * corrected_resting_ratio * 1000.0
	spring_rate = weight / target_compression

func set_damping_rate(weight : float) -> void:
	if spring_rate == 0.0:
		push_error('must init spring rate before initializing damping rate')

	_damping_rate = damping_ratio * 2.0 * sqrt(spring_rate * weight) * 0.01

func set_powered_status(status : bool) -> void:
	is_powered = status
	left_wheel.is_powered = status
	right_wheel.is_powered = status

func process_axle_drive(torque : float, drive_inertia : float, delta : float) -> void:
	if not is_powered:
		torque = 0.0
		drive_inertia = 0.0

	var allow_abs := true

	## If the handbrake in engaged, disable the antilock brakes
	if handbrake:
		brake_force += handbrake_force
		allow_abs = false

	## If enough torque is applied to the axle, lock to wheel speeds and add
	## torque vectoring
	if is_powered and locking_differential_engage_torque >= 0.0:
		if absf(torque) > locking_differential_engage_torque:
			rotation_split = 0.5 + (torque_vectoring * -_vehicle.steering_input)
			var couple_spin := get_average_spin()
			left_wheel.spin = couple_spin * rotation_split * 2.0
			right_wheel.spin = couple_spin * (1.0 - rotation_split) * 2.0
			rotation_split = (rotation_split * 2.0) - 1.0
		elif torque != 0.0:
			var left_reaction_torque_ratio := -absf((left_wheel.get_reaction_torque()) / torque)
			var right_reaction_torque_ratio := absf((right_wheel.get_reaction_torque()) / torque)
			rotation_split = maxf(rotation_split, left_reaction_torque_ratio)
			rotation_split = minf(rotation_split, right_reaction_torque_ratio)

	var rotation_sum := 0.0
	var split := (rotation_split + 1.0) * 0.5
	applied_split = rotation_split
	rotation_sum += left_wheel.process_torque(torque * split, drive_inertia, brake_force * 0.5 * brake_bias, allow_abs, delta)
	rotation_sum += right_wheel.process_torque(torque * (1.0 - split), drive_inertia, brake_force * 0.5 * brake_bias, allow_abs, delta)
	rotation_split = clampf(rotation_sum, -1.0, 1.0)

func calculate_average_tire_friction(weight : float, surface : String) -> float:
	var friction := 0.0
	friction += left_wheel.get_friction(weight / 2, surface)
	friction += right_wheel.get_friction(weight / 2, surface)
	return friction

func calculate_brake_force(average_drive_wheel_radius : float, total_number_of_wheels : int) -> void:
	var friction := calculate_average_tire_friction(_vehicle.mass * 9.8, "Road")
	max_brake_force = ((friction * braking_grip_multiplier) * average_drive_wheel_radius) / total_number_of_wheels
	max_handbrake_force = ((friction * braking_grip_multiplier * 0.05) / average_drive_wheel_radius)
#endregion

#region private functions
#endregion

#region subclasses
#endregion
