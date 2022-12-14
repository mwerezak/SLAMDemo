extends Node2D

const Marker = preload("res://RoverPawn.tscn")

export(int, 0, 5000) var marker_count: int = 100 setget _set_marker_count_deferred
export(int, 0, 5000) var particle_count = 1000
export(Color) var marker_color: Color

onready var _pfilter = $ParticleFilter

var _markers = []
var _update = false

func reset(pose: Transform2D):
	_pfilter.set_particle_count(particle_count)
	_pfilter.reset_pose_with_absolute_certainty(pose)
	_update = true

func motion_update(motion_model):
	_pfilter.motion_update(motion_model)
	_update = true

func gps_update(gps_meas):
	_pfilter.gps_update(gps_meas)
	_update = true

func _ready():
	_set_marker_count(marker_count)

func _process(_delta):
	if _update and self.visible:
		_update_markers()
		_update = false

func _set_marker_count(value):
	while value < _markers.size():
		var marker = _markers.pop_back()
		marker.queue_free()
	while value > _markers.size():
		_create_marker()
	_update = true

func _set_marker_count_deferred(value):
	call_deferred('_set_marker_count', value)
		
func _update_markers():
	var particles = _pfilter.get_particles(_markers.size())
	if particles == null or particles.empty():
		for marker in _markers:
			marker.hide()
		return
	
	var max_weight = particles[0][1]
#	print("max weight: ", max_weight)
#	print("particles", particles)
	
	var i = 0
	for marker in _markers:
		if i >= particles.size():
			marker.hide()
			continue
		
		var pose = particles[i][0]
		var weight = particles[i][1]
		i += 1
		
		if max_weight <= 0:
			marker.modulate.a = 0.5
		else:
			marker.visible = (weight > 0)
			marker.modulate.a = weight/max_weight
		
		if marker.visible:
			marker.global_position = Vector2(pose.x, pose.y)
			marker.global_rotation = pose.z
	

func _create_marker():
	var marker = Marker.instance()
	add_child(marker)
	_markers.append(marker)
	marker.modulate = marker_color
	marker.scale = 0.5*Vector2.ONE
	
