class_name SteeringParameters
extends Resource

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
