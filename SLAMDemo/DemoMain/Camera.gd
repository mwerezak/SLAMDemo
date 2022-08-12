extends Camera2D

export(float) var pan_rate := 1000.0
export(float) var zoom_rate := 0.1
export(float) var max_zoom = 2.5
export(float) var min_zoom = 0.25

onready var rover = $"../Rover"

func _ready():
	pass # Replace with function body.
	print(rover.position)

const pan_action = {
	ui_up = Vector2(0, -1),
	ui_down = Vector2(0, 1),
	ui_left = Vector2(-1, 0),
	ui_right = Vector2(1, 0),
}

func _process(delta):
	var pan = Vector2()
	for action in pan_action.keys():
		if Input.is_action_pressed(action, true):
			pan += pan_action[action]
	translate(pan_rate*delta*pan)

func _input(event):
	if event is InputEventMouseMotion:
		_handle_mouse_pan(event)
	elif event is InputEvent:
		_update_zoom()
		if Input.is_action_pressed('camera_center'):
			global_position = rover.global_position

func _handle_mouse_pan(event: InputEventMouseMotion):
	if Input.is_action_pressed('camera_drag'):
		var pan = -event.relative
		pan.x *= zoom.x
		pan.y *= zoom.y
		translate(pan)

func _update_zoom():
	if Input.is_action_pressed('camera_zoom_in'):
		zoom /= 1 + zoom_rate
	elif Input.is_action_pressed('camera_zoom_out'):
		zoom *= 1 + zoom_rate
	zoom.x = clamp(zoom.x, min_zoom, max_zoom)
	zoom.y = clamp(zoom.y, min_zoom, max_zoom)
