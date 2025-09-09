class_name MVPSuspensionComponent
extends Resource

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
