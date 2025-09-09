# Portions are Copyright (c) 2021 Dechode
# https://github.com/Dechode/Godot-Advanced-Vehicle

class_name MVPBody
extends RigidBody3D

#region constants
const ANGULAR_VELOCITY_TO_RPM := 60.0 / TAU
#endregion

#region export variables
@export_group("Steering")
## The rate that the steering input changes in order to smooth
## out direction changes to the wheel.
## Steering input is between -1 and 1. Speed is in units per second.
@export var steering_speed := 4.25
## The rate that the steering input changes when steering back to center.
## Speed is in units per second.
@export var countersteer_speed := 11.0
## Reduces steering input based on the vehicle's speed.
## Steering speed is divided by the velocity at this magnitude.
## The larger the number, the slower the steering at speed.
@export var steering_speed_decay := 0.20
## Further steering input is prevented if the wheels' lateral slip is greater than this number.
@export var steering_slip_assist := 0.15
## The magnitude to adjust steering toward the direction of travel based on the vehicle's lateral velocity.
@export var countersteer_assist := 0.9
## Steering input is raised to the power of this number.
## This has the effect of slowing steering input near the limits.
@export var steering_exponent := 1.5
## The maximum steering angle in radians.
## [br][br]
## [b]Note:[/b] This property is edited in the inspector in degrees. If you want to use degrees in a script, use [code]deg_to_rad[/code].
@export_range(0, 360, 0.1, "radians_as_degrees") var max_steering_angle := deg_to_rad(40.0)


@export_group("Throttle and Braking")
## The rate the throttle input changes to smooth input.
## Throttle input is between 0 and 1. Speed is in units per second.
@export var throttle_speed := 20.0
## Multiply the throttle speed by this based on steering input.
@export var throttle_steering_adjust := 0.1
## The rate braking input changes to smooth input.
## Braking input is between 0 and 1. Speed is in units per second.
@export var braking_speed := 10.0
## Prevents engine power from causing the tires to slip beyond this value.
## Values below 0 disable the effect.
@export var traction_control_max_slip := 8.0


@export_group("Stability")
## Stablity applies torque forces to the vehicle body when the body angle
## relative to the direction of travel exceeds a threshold.
@export var enable_stability := true
## The yaw angle the vehicle must reach before stability is applied.
## Based on the dot product, 0 being straight, 1 being 90 degrees
@export var stability_yaw_engage_angle := 0.0
## Strength multiplier for the applied yaw correction.
@export var stability_yaw_strength := 6.0
## Additional strength multiplier for a grounded vehicle to overcome traction.
@export var stability_yaw_ground_multiplier := 2.0
## A multiplier for the torque used to keep the vehicle upright while airbourn.
@export var stability_upright_spring := 1.0
## A multiplier for the torque used to dampen rotation while airborne.
@export var stability_upright_damping := 1000.0


@export_group("Motor")
## Maximum motor torque in NM.
@export var max_torque := 300.0
## Maximum motor RPM.
@export var max_rpm := 7000.0
## Idle motor RPM.
@export var idle_rpm := 1000.0
## Percentage of torque produced across the RPM range.
@export var torque_curve : Curve
## Variable motor drag based on RPM.
@export var motor_drag := 0.005
## Constant motor drag.
@export var motor_brake := 10.0
## Moment of inertia for the motor.
@export var motor_moment := 0.5
## The motor will use this rpm when launching from a stop.
@export var clutch_out_rpm := 3000.0
## Max clutch torque as a ratio of max motor torque.
@export var max_clutch_torque_ratio := 1.6


@export_group("Gearbox")
## Transmission gear ratios, the size of the array determines the number of gears
@export var gear_ratios : Array[float] = [ 3.8, 2.3, 1.7, 1.3, 1.0, 0.8 ]
## Final drive ratio
@export var final_drive := 3.2
## Reverse gear ratio
@export var reverse_ratio := 3.3
## Time it takes to change gears on up shifts in seconds
@export var shift_time := 0.3
## Enables automatic gear changes
@export var automatic_transmission := true
## Timer to prevent the automatic gear shifts changing gears too quickly
## in milliseconds
@export var automatic_time_between_shifts := 1000.0
## Drivetrain inertia
@export var gear_inertia := 0.02


@export_group("Drivetrain")
## Torque delivered to the front wheels vs the rear wheels.
## Value of 1 is FWD, a value of 0 is RWD, anything in between is AWD.
@export var front_torque_split := 0.0
## When enabled, the torque split will change based on wheel slip.
@export var variable_torque_split := false
## Torque split to interpolate toward when there is wheel slip. Variable Torque
## Split must be enabled.
@export var front_variable_split := 0.0
## How quickly to interpolate between torque splits in seconds.
@export var variable_split_speed := 1.0

@export_group("Suspension")
## The percentage of the vehicle mass over the front axle.
@export_range(0.0, 1.0) var front_weight_distribution := 0.5
## The center of gravity is calculated from the front weight distribution
## with the height centered on the wheel raycast positions. This will offset
## the height from that calculated position.
@export var center_of_gravity_height_offset := -0.2
## Multiplies the calculated inertia by this value.
## Greater inertia values will cause more force to be
## required to rotate the car.
@export var inertia_multiplier := 1.2

@export_group("Aerodynamics")
## The drag coefficient quantifies how much [b]drag[/b] (force against thrust)
## the vehicle recieves when moving through air. In the drag equation,
## a lower drag coefficient means the vehicle will experience less drag
## force, allowing it to move faster.
## [br]Typically, the drag coefficient is assumed from the shape of the
## body, where more teardrop-shaped bodies experience a lower drag coefficient.
## Un-streamlined cyllindrical bodies have a drag coefficient of
## around [code]0.80[/code], while more streamlined teardrop-shaped bodies
## can have a drag coefficient as low as [code]0.05[/code], or even lower.
## [br]As a more relavant example, most cars have drag coefficients
## around [code]0.40[/code].
@export var coefficient_of_drag := 0.3
## From [url=https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/density.html#:~:text=Halving%20the%20density%20halves%20the,above%20which%20it%20cannot%20fly.]NASA[/url]:
## [i]"Halving the density halves the lift, halving the density halves the drag. The [lb]air[rb] density depends on the type of [lb]air[rb] and the depth of the [lb]air[rb]. In the atmosphere, air density decreases as altitude increases. This explains why airplanes have a flight ceiling, an altitude above which it cannot fly."[/i]
@export var air_density := 1.225
## The amount of surface area the front-facing part of the vehicle has,
## in meters squared ([code]m^2[/code]).
## [br][br]
## [b]Note:[/b] You do not have to calculate this value to be exact,
## a rough estimate - or even something completely different, depending
## on the result you want - will do.
@export var frontal_area := 2.0
#endregion

#region public variables
## Controller Inputs: An external script should set these values
var throttle_input := 0.0
var steering_input := 0.0
var brake_input := 0.0
var handbrake_input := 0.0
var clutch_input := 0.0
#endregion

#region private variables
var _powered_wheels : Array[MVPWheel] = []
var _steering_wheels : Array[MVPWheel] = []
var _wheel_array : Array[MVPWheel] = []
var _axles : Array[MVPAxle] = []

var is_ready := false
var local_velocity := Vector3.ZERO
var previous_global_position := Vector3.ZERO
var speed := 0.0
var motor_rpm := 0.0

var steering_amount := 0.0
var steering_exponent_amount := 0.0
var true_steering_amount := 0.0
var throttle_amount := 0.0
var brake_amount := 0.0
var clutch_amount := 0.0

var current_gear := 0
var requested_gear := 0

var torque_output := 0.0
var clutch_torque := 0.0
var max_clutch_torque := 0.0
var drive_axles_inertia := 0.0
var complete_shift_delta_time := 0.0
var last_shift_delta_time := 0.0
var average_drive_wheel_radius := 0.0
var current_torque_split := 0.0
var true_torque_split := 0.0
var is_braking := false
var motor_is_redline := false
var is_shifting := false
var is_up_shifting := false
var need_clutch := false
var tcs_active := false
var stability_active := false
var stability_yaw_torque := 0.0
var stability_torque_vector := Vector3.ZERO
var front_axle_position := Vector3.ZERO
var rear_axle_position := Vector3.ZERO

var delta_time := 0.0

var vehicle_inertia : Vector3
var current_gravity : Vector3
#endregion

#region overridden built-in virtual functions
func _ready()-> void:
	pass

func _physics_process(delta : float) -> void:
	if not is_ready:
		return

	## For stability calculations, we need the vehicle body inertia which isn't
	## available immediately
	if not vehicle_inertia:
		var rigidbody_inertia := PhysicsServer3D.body_get_direct_state(get_rid()).inverse_inertia.inverse()
		if rigidbody_inertia.is_finite():
			vehicle_inertia = rigidbody_inertia * inertia_multiplier
			inertia = vehicle_inertia

	delta_time += delta
	local_velocity = lerp(((global_transform.origin - previous_global_position) / delta) * global_transform.basis, local_velocity, 0.5)
	previous_global_position = global_position
	speed = local_velocity.length()

	process_drag()
	process_braking(delta)
	process_steering(delta)
	process_throttle(delta)

	process_motor(delta)
	process_clutch(delta)
	process_transmission()
	process_drive(delta)
	process_stability()

func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	current_gravity = state.total_gravity
#endregion

#region public functions
func initialize() -> void:
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	var center_of_gravity := calculate_center_of_gravity(front_weight_distribution)
	center_of_gravity.y += center_of_gravity_height_offset
	center_of_mass = center_of_gravity
	max_clutch_torque = max_torque * max_clutch_torque_ratio

	average_drive_wheel_radius /= _powered_wheels.size()
	previous_global_position = global_position

	is_ready = true

func process_drag() -> void:
	var drag := 0.5 * air_density * pow(speed, 2.0) * frontal_area * coefficient_of_drag
	if drag > 0.0:
		apply_central_force(-local_velocity.normalized() * drag)

func process_braking(delta : float) -> void:
	if (brake_input < brake_amount):
		brake_amount -= braking_speed * delta
		if (brake_input > brake_amount):
			brake_amount = brake_input

	elif (brake_input > brake_amount):
		brake_amount += braking_speed * delta
		if (brake_input < brake_amount):
			brake_amount = brake_input

	if brake_amount > 0.0:
		is_braking = true
	else:
		is_braking = false

func process_steering(delta : float) -> void:
	var steer_assist_engaged := false
	var steering_slip := get_max_steering_slip_angle()

	## Adjust steering speed based on vehicle speed and max steering angle
	var steer_speed_correction := steering_speed / (speed * steering_speed_decay) / max_steering_angle

	## If the steering input is opposite the current steering, apply countersteering speed instead
	if signf(steering_input) != signf(steering_amount):
		steer_speed_correction = countersteer_speed / (speed * steering_speed_decay)

	## Check steering slip threshold and reduce steering amount if crossed.
	if absf(steering_slip) > steering_slip_assist:
		steer_assist_engaged = true

	if (steering_input < steering_amount):
		if not steer_assist_engaged or steering_slip < 0.0:
			steering_amount -= steer_speed_correction * delta
			if (steering_input > steering_amount):
				steering_amount = steering_input
		else:
			steering_amount += steer_speed_correction * delta
			if (steering_amount > 0.0):
				steering_amount = 0.0

	elif (steering_input > steering_amount):
		if not steer_assist_engaged or steering_slip > 0.0:
			steering_amount += steer_speed_correction * delta
			if (steering_input < steering_amount):
				steering_amount = steering_input
		else:
			steering_amount -= steer_speed_correction * delta
			if (steering_amount < 0.0):
				steering_amount = 0.0

	## Steering exponent adjustment
	var steering_adjust := pow(absf(steering_amount), steering_exponent) * signf(steering_amount)

	## Correct steering toward the direction of travel for countersteer assist
	var steer_correction := (1.0 - absf(steering_adjust)) * clampf(asin(local_velocity.normalized().x), -max_steering_angle, max_steering_angle) * countersteer_assist

	## Don't apply corrections at low velocity or reversing
	if local_velocity.z > -0.5:
		steer_correction = 0
	else:
		steer_correction = steer_correction / -max_steering_angle

	## Keeps steering corrections from getting stuck under certain circumstances
	var steer_correction_amount := 1.0
	if signf(steering_adjust + steer_correction) != signf(steering_input) and 1.0 - absf(steering_input) < steer_correction_amount:
		steer_correction_amount = clampf(steer_correction_amount - (steering_speed * delta), 0.0, 1.0)
	else:
		steer_correction_amount = clampf(steer_correction_amount + (steering_speed * delta), 0.0, 1.0)

	steer_correction *= steer_correction_amount

	true_steering_amount = clampf(steering_adjust + steer_correction, -max_steering_angle, max_steering_angle)

	for wheel in _wheel_array:
		wheel.steer(steering_adjust + steer_correction, max_steering_angle)

func process_throttle(delta : float) -> void:
	var throttle_delta := throttle_speed * delta

	if (throttle_input < throttle_amount):
		throttle_amount -= throttle_delta
		if (throttle_input > throttle_amount):
			throttle_amount = throttle_input

	elif (throttle_input >= throttle_amount):
		throttle_amount += throttle_delta
		if (throttle_input < throttle_amount):
			throttle_amount = throttle_input

	## Cut throttle at redline and when shifting
	if motor_is_redline or is_shifting:
		throttle_amount = 0.0

	## Disengage clutch when shifting or below motor idle
	if need_clutch or is_shifting:
		clutch_amount = 1.0
	else:
		clutch_amount = clutch_input

func process_motor(delta : float) -> void:
	var drag_torque := motor_rpm * motor_drag
	torque_output = get_torque_at_rpm(motor_rpm) * throttle_amount
	## Adjust torque based on throttle input, clutch input, and motor drag
	torque_output -= drag_torque * (1.0 + (clutch_amount * (1.0 - throttle_amount)))

	## Prevent motor from outputing torque below idle or far beyond redline
	var new_rpm := motor_rpm
	new_rpm += ANGULAR_VELOCITY_TO_RPM * delta * torque_output / motor_moment
	motor_is_redline = false
	if new_rpm > max_rpm * 1.1 or new_rpm <= idle_rpm:
		torque_output = 0.0
		if new_rpm > max_rpm * 1.1:
			motor_is_redline = true

	motor_rpm += ANGULAR_VELOCITY_TO_RPM * delta * (torque_output - drag_torque) / motor_moment

	## Disengage clutch when near idle
	if motor_rpm < idle_rpm + 100:
		need_clutch = true
	elif new_rpm > clutch_out_rpm:
		need_clutch = false

	motor_rpm = maxf(motor_rpm, idle_rpm)

func process_clutch(delta : float) -> void:
	if current_gear == 0:
		return

	## Formula to calculate the forces needed to keep the drivetrain and motor closely coupled
	var current_gear_ratio := get_gear_ratio(current_gear)
	var drive_inertia := motor_moment + (pow(absf(current_gear_ratio), 2.0) * gear_inertia) + drive_axles_inertia
	var drive_inertia_R := drive_inertia / (current_gear_ratio * current_gear_ratio)
	var reaction_torque := get_powered_wheels_reaction_torque() / current_gear_ratio
	var speed_difference := (motor_rpm / ANGULAR_VELOCITY_TO_RPM) - (get_drivetrain_spin() * current_gear_ratio)
	if speed_difference < 0.0:
		speed_difference = -sqrt(-speed_difference)
	var a := (motor_moment * drive_inertia_R * speed_difference) / delta
	var b := motor_moment * reaction_torque
	var c := drive_inertia_R * torque_output
	var clutch_factor := (1.0 - clutch_amount)
	var tcs_torque_reduction := 0.0
	clutch_torque = ((a - b + c)/(motor_moment + drive_inertia_R)) * clutch_factor
	clutch_torque = clampf(clutch_torque, -max_clutch_torque * clutch_factor, max_clutch_torque * clutch_factor)

	## Check if traction control is needed and adjust motor speed and clutch torque if needed
	if traction_control_max_slip > 0.0:
		var slip_y := 0.0
		for axle in _axles:
			slip_y = maxf(slip_y, axle.get_max_wheel_slip_y())
		if slip_y > traction_control_max_slip:
			tcs_torque_reduction = torque_output
			clutch_torque = 0.0
			tcs_active = true
		else:
			tcs_active = false

	var clutch_reaction_torque := clutch_torque + tcs_torque_reduction
	var new_rpm := motor_rpm - ((ANGULAR_VELOCITY_TO_RPM * delta * clutch_reaction_torque) / motor_moment)
	if new_rpm < idle_rpm:
		new_rpm = idle_rpm
	if new_rpm < idle_rpm + 100:
		need_clutch = true
	elif new_rpm > clutch_out_rpm:
		need_clutch = false
	if new_rpm > max_rpm * 1.1:
		new_rpm = max_rpm * 1.1

	motor_rpm = new_rpm

func process_transmission() -> void:
	if is_shifting:
		if delta_time > complete_shift_delta_time:
			complete_shift()
		return

	## For automatic transmissions to determine when to shift the current wheel speed and
	## what the wheel speed would be without slip are used. This allows vehicles to spin the
	## tires without immidiately shifting to the next gear.

	if automatic_transmission:
		var reversing := false
		var ideal_wheel_spin := speed / average_drive_wheel_radius
		var drivetrain_spin := get_drivetrain_spin()
		var real_wheel_spin := drivetrain_spin * get_gear_ratio(current_gear)
		var current_ideal_gear_rpm := gear_ratios[current_gear - 1] * final_drive * ideal_wheel_spin * ANGULAR_VELOCITY_TO_RPM
		var current_real_gear_rpm := real_wheel_spin * ANGULAR_VELOCITY_TO_RPM

		if current_gear == -1:
			reversing = true

		if not reversing:
			var previous_gear_rpm := 0.0
			if current_gear - 1 > 0:
				previous_gear_rpm = get_gear_ratio(current_gear - 1) * maxf(drivetrain_spin, ideal_wheel_spin) * ANGULAR_VELOCITY_TO_RPM


			if current_gear < gear_ratios.size():
				if current_gear > 0:
					if current_ideal_gear_rpm > max_rpm:
						if delta_time - last_shift_delta_time > shift_time:
							shift(1)
					if current_ideal_gear_rpm > max_rpm * 0.8 and current_real_gear_rpm > max_rpm:
						if delta_time - last_shift_delta_time > shift_time:
							shift(1)
				elif current_gear == 0 and motor_rpm > clutch_out_rpm:
					shift(1)
			if current_gear - 1 > 0:
				if current_gear > 1 and previous_gear_rpm < 0.75 * max_rpm:
					if delta_time - last_shift_delta_time > shift_time:
						shift(-1)

		if absf(current_gear) <= 1 and brake_input > 0.75:
			if not reversing:
				if speed < 1.0 or local_velocity.z > 0.0:
					if delta_time - last_shift_delta_time > shift_time:
						shift(-1)
			else:
				if speed < 1.0 or local_velocity.z < 0.0:
					if delta_time - last_shift_delta_time > shift_time:
						shift(1)

func process_drive(_delta : float) -> void:
	pass

func process_stability() -> void:
	var is_stability_on := false
	## Calculates the angle of the vehicle in relation to the direction of travel
	## and applies necessary stabilizing forces.
	if enable_stability:
		stability_yaw_torque = 0.0
		var plane_xz := Vector2(local_velocity.x, local_velocity.z)
		if plane_xz.y < 0 and plane_xz.length() > 1.0:
			plane_xz = plane_xz.normalized()
			var yaw_angle := 1.0 - absf(plane_xz.dot(Vector2.UP))
			if yaw_angle > stability_yaw_engage_angle and signf(angular_velocity.y) == signf(plane_xz.x):
				stability_yaw_torque = (yaw_angle - stability_yaw_engage_angle) * stability_yaw_strength
				stability_yaw_torque *= vehicle_inertia.y * clampf(absf(angular_velocity.y) - 0.5, 0.0, 1.0)

		stability_torque_vector = Vector3.ZERO
		if get_wheel_contact_count() < 3:
			stability_torque_vector = (global_transform.basis.y.cross(Vector3.UP) * vehicle_inertia * stability_upright_spring) + (-angular_velocity * stability_upright_damping)
			apply_torque(stability_torque_vector)
		else:
			stability_yaw_torque *= stability_yaw_ground_multiplier

		if stability_yaw_torque:
			is_stability_on = true
			stability_yaw_torque *= signf(-local_velocity.x)
			apply_torque(global_transform.basis.y * stability_yaw_torque)

	stability_active = is_stability_on

func manual_shift(count : int) -> void:
	if not automatic_transmission:
		shift(count)

func shift(count : int) -> void:
	if is_shifting:
		return

	## Handles gear change requests and timings
	requested_gear = current_gear + count

	if requested_gear <= gear_ratios.size() and requested_gear >= -1:
		if current_gear == 0:
			complete_shift()
		else:
			complete_shift_delta_time = delta_time + shift_time
			clutch_amount = 1.0
			is_shifting = true
			if count > 0:
				is_up_shifting = true

func complete_shift() -> void:
	## Called when it is time to complete a shift in progress
	if current_gear == -1:
		brake_amount = 0.0
	if requested_gear < current_gear:
		var wheel_spin := speed / average_drive_wheel_radius
		var requested_gear_rpm := gear_ratios[requested_gear - 1] * final_drive * wheel_spin * ANGULAR_VELOCITY_TO_RPM
		motor_rpm = lerpf(motor_rpm, requested_gear_rpm, 0.5)
	current_gear = requested_gear
	last_shift_delta_time = delta_time
	is_shifting = false
	is_up_shifting = false

func get_wheel_contact_count() -> int:
	var contact_count := 0
	for wheel in _wheel_array:
		if wheel.is_colliding():
			contact_count += 1
	return contact_count

func get_is_a_wheel_slipping() -> bool:
	var is_slipping := false
	for wheel in _powered_wheels:
		if not wheel.limit_spin:
			is_slipping = true
	return is_slipping

func get_drivetrain_spin() -> float:
	if _powered_wheels.size() == 0:
		return 0.0

	var drive_spin := 0.0
	for wheel in _powered_wheels:
		drive_spin += wheel.spin

	return drive_spin / _powered_wheels.size()

func get_powered_wheels_reaction_torque() -> float:
	var reaction_torque := 0.0
	for wheel in _powered_wheels:
		reaction_torque += wheel.force_vector.y * wheel.get_tire_radius()
	return reaction_torque

func get_gear_ratio(gear : int) -> float:
	if gear > 0:
		return gear_ratios[gear - 1] * final_drive
	elif gear == -1:
		return -reverse_ratio * final_drive
	else:
		return 0.0

func get_torque_at_rpm(lookup_rpm : float) -> float:
	var rpm_factor := clampf(lookup_rpm / max_rpm, 0.0, 1.0)
	var torque_factor : float = torque_curve.sample_baked(rpm_factor)
	return torque_factor * max_torque

func get_max_steering_slip_angle() -> float:
	var steering_slip := 0.0
	for wheel in _steering_wheels:
		if absf(steering_slip) < absf(wheel.slip_vector.x):
			steering_slip = wheel.slip_vector.x
	return steering_slip

func calculate_average_tire_friction(weight : float, surface : String) -> float:
	var friction := 0.0
	for wheel in _wheel_array:
		friction += wheel.get_friction(weight / _wheel_array.size(), surface)
	return friction

func calculate_center_of_gravity(_front_distribution : float) -> Vector3:
	return Vector3(0,0,0)
#endregion

#region subclasses
#endregion
