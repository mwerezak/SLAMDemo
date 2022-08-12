class_name Rover
extends Node2D

export(float) var max_speed := 200.0
export(float) var max_accel := 100.0
export(float) var max_brake := 500.0
export(float) var rotation_speed_degrees setget _set_rotation_speed_degress

var rotation_speed: float = deg2rad(90)  # rad/s
var _cur_speed := 0.0

func _set_rotation_speed_degress(value: float):
	rotation_speed = deg2rad(value)


var speed: float setget , get_speed

func get_speed() -> float:
	return _cur_speed


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

const _control_update := {
	rover_fwd = Vector2(0, 1),
	rover_rev = Vector2(0, -1),
	rover_left = Vector2(-1, 0),
	rover_right = Vector2(1, 0),
}

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	var control := Vector2()
	for action in _control_update.keys():
		if Input.is_action_pressed(action, true):
			control += _control_update[action]
	
	_motion_update(control, delta)

func _motion_update(control: Vector2, delta: float):
	var turn_cmd := control.x
	var spd_cmd := control.y
	
	rotate(rotation_speed*sign(turn_cmd)*delta)
	
	var tgt_speed := max_speed * spd_cmd
	var accel := tgt_speed - _cur_speed
	if _cur_speed > 0:
		accel = clamp(accel, -max_brake, max_accel)
	elif _cur_speed < 0:
		accel = clamp(accel, -max_accel, max_brake)
	else:
		accel = clamp(accel, -max_accel, max_accel)
	_cur_speed = clamp(_cur_speed + accel*delta, -max_speed, max_speed)
	translate(delta*_cur_speed*Vector2.RIGHT.rotated(rotation))
	
