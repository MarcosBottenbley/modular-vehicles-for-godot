class_name MVPStabilityComponent
extends Resource

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
