extends Node2D

const Marker = preload("res://RoverPawn.tscn")

export(int, 0, 10000) var marker_count: int = 100
export(int, 0, 10000) var particle_count: int = 1000 setget _set_particle_count
export(Color) var marker_color: Color

onready var _pfilter = $ParticleFilter

var _markers = []
var _update = false

func reset(pose: Transform2D):
	_pfilter.reset_pose_with_absolute_certainty(pose)
	_update = true

func motion_update(motion_model):
	_pfilter.motion_update(motion_model)
	_update = true

func gps_update(gps_meas):
	_pfilter.gps_update(gps_meas)
	_update = true

func _ready():
	_set_particle_count(particle_count)
	for _i in range(marker_count):
		_create_marker()

func _process(_delta):
	if _update and self.visible:
		_update_markers()
		_update = false

func _set_particle_count(value):
	if _pfilter != null:
		_pfilter.set_particle_count(value)
		print(_pfilter.get_particle_count())
		
		
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
	
