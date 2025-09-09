class_name MVPEngineComponent
extends Resource

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

func get_torque_at_rpm(lookup_rpm : float) -> float:
	var rpm_factor := clampf(lookup_rpm / max_rpm, 0.0, 1.0)
	var torque_factor : float = torque_curve.sample_baked(rpm_factor)
	return torque_factor * max_torque
