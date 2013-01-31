define puck model
	(
		size [ 0.08 0.08 0.08]
		gui_nose 0
		boundary 0
		ranger_return 0
		laser_return 0
		blob_return 1
		gripper_return 1
		obstacle_return 0 # used to avoid hit the box when getting away from this one (FIX IT)
		mass 1
	)
#Marcos a serem detectados...
define orange model
(
	# this is a picture of a black circle
	bitmap "bitmaps/circle.png"
	size [ 0.2 0.2 0.2]
	color "black"
	gui_nose 0
	boundary 0
	ranger_return 0
	laser_return 0
	blob_return 1
	gripper_return 0
	obstacle_return 0 # used to avoid hit the box when getting away from this one (FIX IT)
	mass 1
)


