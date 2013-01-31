# The Pioneer3DX sonar array
define p3dx_sonar ranger
(
  scount 16

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.069 0.136 90 ]
  spose[1] [ 0.114 0.119 50 ]
  spose[2] [ 0.148 0.078 30 ]
  spose[3] [ 0.166 0.027 10 ]
  spose[4] [ 0.166 -0.027 -10 ]
  spose[5] [ 0.148 -0.078 -30 ]
  spose[6] [ 0.114 -0.119 -50 ]
  spose[7] [ 0.069 -0.136 -90 ]
  spose[8] [ -0.157 -0.136 -90 ]
  spose[9] [ -0.203 -0.119 -130 ]
  spose[10] [ -0.237 -0.078 -150 ]
  spose[11] [ -0.255 -0.027 -170 ]
  spose[12] [ -0.255 0.027 170 ]
  spose[13] [ -0.237 0.078 150 ]
  spose[14] [ -0.103 0.119 130 ]
  spose[15] [ -0.157 0.136 90 ]	
		
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0.1 5.0 30]  # min (m), max (m), field of view (deg)

  # define the size of each transducer [xsize ysize] in meters
  ssize [0.01 0.04]
  laser_return 0 
  obstacle_return 0 
)

define pioneer_base position 
(
  color "red"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 0           	# Can hit things.
  laser_return 0                # reflects laser beams
  ranger_return 0             	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  #localization "gps"         	# Change to "gps" to have impossibly perfect, global odometry  
  #localization_origin [0 0 0 0] # Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  localization "odom"         		
  odom_error [ 0.03 0.03 0.1 ]   	# Odometry error or slip in X, Y and Theta
                               	# (Uniform random distribution)   
)

# The Pioneer3DX standard configuration
define pioneer3dx pioneer_base
(
  # Actual size
  size [0.511 0.4 0.22 ]

  # The pioneer's center of rotation is offset from its center of area
  origin [-0.04465 0.0 0.0]

  # Estimated mass in KG
  mass 23.0 

  # Body shape:
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )

  # Use the sonar array defined above
  p3dx_sonar( pose [0 0 -0.03 0] )  
  laser_return 0 
  obstacle_return 0 
) 

