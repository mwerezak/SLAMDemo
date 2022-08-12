extends Resource
class_name OdometryNoiseModel


export(float, 0, 1, 0.00000001) var rot_rot: float     = 0.1      # effect of rotation speed on rotation noise
export(float, 0, 1, 0.00000001) var trans_rot: float   = 0.000005 # effect of translation speed on rotation noise
export(float, 0, 1, 0.00000001) var trans_trans: float = 0.0001   # effect of translation speed on translation noise
export(float, 0, 1, 0.00000001) var rot_trans: float   = 0.01     # effect of rotation speed on translation noise
