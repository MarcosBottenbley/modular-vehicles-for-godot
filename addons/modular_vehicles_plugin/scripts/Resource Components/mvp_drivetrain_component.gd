class_name MVPDrivetrainComponent
extends Resource

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
