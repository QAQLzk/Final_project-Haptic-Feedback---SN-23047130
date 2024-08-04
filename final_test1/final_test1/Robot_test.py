
import URBasic
import math
import numpy as np
import sys
# import cv2
import time
# import imutils
# from imutils.video import VideoStream
import math3d as m3d

ROBOT_IP = '192.168.1.121'
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

# The Joint position the robot starts at
robot_startposition = (math.radians(0),
                    math.radians(-115),
                    math.radians(120),
                    math.radians(-180),
                    math.radians(-88),
                    math.radians(0))

# Maximum Rotation of the robot at the edge of the view window
hor_rot_max = math.radians(50)
vert_rot_max = math.radians(25)
# Variable which scales the robot movement from pixels to meters.
m_per_pixel = 00.00009  
# Size of the robot view-window

# The robot will at most move this distance in each direction
max_x = 0.2
max_y = 0.2

# initialise robot with URBasic
print("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)

robot.reset_error()
print("robot initialised")
time.sleep(1)

# Move Robot to the midpoint of the lqookplane
robot.movej(q=robot_startposition, a= ACCELERATION, v= VELOCITY )

robot_position = [0,0]
robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised

