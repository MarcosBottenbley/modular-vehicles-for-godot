class_name MVPGearboxComponent
extends Resource

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

func get_gear_ratio(gear : int) -> float:
	if gear > 0:
		return gear_ratios[gear - 1] * final_drive
	elif gear == -1:
		return -reverse_ratio * final_drive
	else:
		return 0.0
