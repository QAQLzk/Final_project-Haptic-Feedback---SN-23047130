
import URBasic
import math
import numpy as np
import sys
# import cv2
import time
# import imutils
# from imutils.video import VideoStream
import math3d as m3d


def UR_initial(robot_startposition):

    
    ROBOT_IP = '192.168.1.121'
    ACCELERATION = 0.9  # Robot acceleration value
    VELOCITY = 0.8  # Robot speed value


    # Size of the robot view-window

    # The robot will at most move this distance in each direction


    # initialise robot with URBasic
    print("initialising robot")
    robotModel = URBasic.robotModel.RobotModel()
    robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)

    robot.reset_error()
    print("robot initialised")
    time.sleep(1)

    # Move Robot to the midpoint of the lookplane
    robot.movej(q=robot_startposition, a= ACCELERATION, v= VELOCITY )


    robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
    time.sleep(1) # just a short wait to make sure everything is initialised


    return robot



