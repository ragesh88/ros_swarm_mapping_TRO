# Desc: Device definitions for Activemedia robots.
# Author: Richard Vaughan, Andrew Howard,  Luis Riazuelo
# Date: 10 Jun 2002
# CVS: $Id: pioneer.inc,v 1.30 2008-01-15 01:25:42 rtv Exp $

# The Pioneer2DX sonar array

#  prototype for p2dx sonar sensors
define p2dxsonar sensor
(
  # define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.05 0.01 ] 
  # define the range bounds [min max]
  range [0 5.0]
  # define the angular field of view in degrees
  fov 15
  # define the number of samples spread over the fov
  samples 1
)

define p2dx_sonar ranger
(
 # 16 transducers spread about the robot as follows
 p2dxsonar( pose [ 0.075 0.130 0 90 ] )
 p2dxsonar( pose [ 0.115 0.115 0 50 ] )
 p2dxsonar( pose [ 0.150 0.080 0 30 ] )
 p2dxsonar( pose [ 0.170 0.025 0 10 ] )
 p2dxsonar( pose [ 0.170 -0.025 0 -10 ] )
 p2dxsonar( pose [ 0.150 -0.080 0 -30 ] )
 p2dxsonar( pose [ 0.115 -0.115 0 -50 ] )
 p2dxsonar( pose [ 0.075 -0.130 0 -90 ] )
 p2dxsonar( pose [ -0.155 -0.130 0 -90 ] )
 p2dxsonar( pose [ -0.195 -0.115 0 -130 ] )
 p2dxsonar( pose [ -0.230 -0.080 0 -150 ] )
 p2dxsonar( pose [ -0.250 -0.025 0 -170 ] )
 p2dxsonar( pose [ -0.250 0.025 0 170 ] )
 p2dxsonar( pose [ -0.230 0.080 0 150 ] )
 p2dxsonar( pose [ -0.195 0.115 0 130 ] )
 p2dxsonar( pose [ -0.155 0.130 0 90 ]	 )
)




define pioneer_base position 
(
  color "red"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 0           	# Can hit things.
  ranger_return 0.5            	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  
  localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)   

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]				
  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]					
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]
)



define pioneer2dx_base_no_sonar pioneer_base
(
  # actual size
  size [0.44 0.38 0.22] # sizes from MobileRobots' web site

  # the pioneer's center of rotation is offset from its center of area
  origin [-0.04 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 23.0 
  # differential steering model
  drive "diff"
 )


define pioneer2dx_base pioneer2dx_base_no_sonar
(
  # use the sonar array defined above with a small vertical offset to
  # drop the sensors into the robot body
  # p2dx_sonar( pose [0 0 -0.03 0] ) 
)




define pioneer2dx pioneer2dx_base
(  
  # simplified Body shape:
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555 ]

    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )
)


