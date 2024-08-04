import ctypes, ctypes.util
import time
import sys
import select
import URobotic
from collections import deque
import math
import math3d as m3d
import keyboard
import serial
import threading
from keras.models import load_model
import os

# The path of the model
file_path = os.path.join(os.path.dirname(__file__), '../../data collection_2 (Z)/Test/Z_best_model.h5')

# Check if the file exists
if os.path.exists(file_path):
    model = load_model(file_path)
else:
    print("File not found.")



CONTROL_FREQUENCY = 125  # Robotic arm (Hz)
HAPTIC_FREQUENCY = 1000  # haptic freqency (Hz)
TIME_INTERVAL = 1 / CONTROL_FREQUENCY  # Time interver (s)

robot_startposition = (math.radians(0),
                math.radians(-90),
                math.radians(90),
                math.radians(-90),
                math.radians(-90),
                math.radians(0))

global scale, robot_max_range
scale = 1
haptic_max_range = {'x': 0.055, 'y': 0.080, 'z': 0.075}  # unit: m
robot_max_range = {'x': 0.1 * scale, 'y': 0.1 * scale, 'z': 0.1 * scale}  # unit:m

#DRD

drd_path = ctypes.util.find_library("drd64.dll")
drd_dll = ctypes.CDLL(drd_path)
#drdIsInitialized
drd_dll.drdIsInitialized.restype = ctypes.c_bool
drd_dll.drdIsInitialized.argtypes = []
#drdAutoInit
drd_dll.drdAutoInit.restype = ctypes.c_int
drd_dll.drdAutoInit.argtypes = []
# drdRegulatePos
drd_dll.drdRegulatePos.restype = ctypes.c_int
drd_dll.drdRegulatePos.argtypes = [ctypes.c_bool]
# drdRegulateRot
drd_dll.drdRegulateRot.restype = ctypes.c_int
drd_dll.drdRegulateRot.argtypes = [ctypes.c_bool]
# drdRegulateGrip
drd_dll.drdRegulateGrip.restype = ctypes.c_int
drd_dll.drdRegulateGrip.argtypes = [ctypes.c_bool]
#drdStart
drd_dll.drdStart.restype = ctypes.c_int
drd_dll.drdStart.argtypes = []
#drdMoveTo
drd_dll.drdMoveTo.restype = ctypes.c_int
drd_dll.drdMoveTo.argtypes = [ctypes.POINTER(ctypes.c_double * 8), ctypes.c_bool, ctypes.c_byte]
#drdClose
drd_dll.drdClose.restype = ctypes.c_int
drd_dll.drdClose.argtypes = []
#drdStop
drd_dll.drdStop.restype = ctypes.c_int
drd_dll.drdStop.argtypes = [ctypes.c_bool]
#drdOpen
drd_dll.drdOpen.restype = ctypes.c_int
#drdIsSupported
drd_dll.drdIsSupported.restype = ctypes.c_bool
#drdGetPositionAndOrientation
drd_dll.drdGetPositionAndOrientation.restype = ctypes.c_int
drd_dll.drdGetPositionAndOrientation.argtypes = [
    ctypes.POINTER(ctypes.c_double),  # px
    ctypes.POINTER(ctypes.c_double),  # py
    ctypes.POINTER(ctypes.c_double),  # pz
    ctypes.POINTER(ctypes.c_double),  # oa
    ctypes.POINTER(ctypes.c_double),  # ob
    ctypes.POINTER(ctypes.c_double),  # og
    ctypes.POINTER(ctypes.c_double),  # pg
    ctypes.POINTER((ctypes.c_double * 3) * 3),  # matrix
    ctypes.c_char  # ID
]
#drdGetVelocity.restype
drd_dll.drdGetVelocity.restype = ctypes.c_int
drd_dll.drdGetVelocity.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
                               ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
                               ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
                               ctypes.POINTER(ctypes.c_double), ctypes.c_char]

drd_dll.drdSetForceAndTorqueAndGripperForce.restype = ctypes.c_int
drd_dll.drdSetForceAndTorqueAndGripperForce.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                                    ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                                    ctypes.c_double, ctypes.c_char]


####################
###Initialization###
####################


##Open the device
if drd_dll.drdOpen() < 0:
    print("error: failed to open device")
    time.sleep(2)
    exit(-1)

# Check if the device is compatible with the robotics library (DRD)
if not drd_dll.drdIsSupported():
    device_name = dhd_dll.dhdGetSystemName().decode('utf-8')
    print(f"unsupported device type {device_name}")
    print("exiting...")
    time.sleep(2.0)
    drd_dll.drdClose()
    sys.exit(-1)


print("haptic initialization successful")



# Variables for position
px, py, pz = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
oa, ob, og, pg = ctypes.c_double(), ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
matrix = ((ctypes.c_double * 3) * 3)()
lastDisplayUpdateTime = time.time()  # initial time
haptic_pos =[]

# Variables for velocity
vx, vy, vz = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
wx, wy, wz, vg = ctypes.c_double(), ctypes.c_double(), ctypes.c_double(), ctypes.c_double()

# Damping coefficients
damping_coefficient_translation =0
damping_coefficient_rotation = 0

# Apply zero force in initialisation
drd_dll.drdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, 0.0, ctypes.c_char(b'\xff') )



#################################################
#Initialise Robotic Arm
robot = URobotic.UR_initial(robot_startposition)


###################################################
buffer = deque(maxlen=HAPTIC_FREQUENCY // CONTROL_FREQUENCY)

running = True
def get_haptic_input():
    # Capture the haptic device position
    pos_result = drd_dll.drdGetPositionAndOrientation(
        ctypes.byref(px),
        ctypes.byref(py),
        ctypes.byref(pz),
        ctypes.byref(oa),
        ctypes.byref(ob),
        ctypes.byref(og),
        ctypes.byref(pg),
        ctypes.byref(matrix),
        ctypes.c_char(b'\xff')
    )

    return [px.value, py.value, pz.value]


def haptic_input_thread():
    while running:
        haptic_input = get_haptic_input()
        if haptic_input:
            buffer.append(haptic_input)
        time.sleep(1 / HAPTIC_FREQUENCY)


current_pose = robot.get_actual_tcp_pose()  
current_position = current_pose[:3]  # Postion (x, y, z)
current_orientation = current_pose[3:]  # Orientation (rx, ry, rz)

###################################################
#define the scale functions

def increase_scale():
    global scale, robot_max_range
    scale = min(3, scale + 0.1)
    robot_max_range = {'x': 0.1 * scale, 'y': 0.1 * scale, 'z': 0.1 * scale}

def decrease_scale():
    global scale, robot_max_range
    scale = max(0.5, scale - 0.1)
    robot_max_range = {'x': 0.1 * scale, 'y': 0.1 * scale, 'z': 0.1 * scale}




################################################################
# Read Sensor Data

latest_serial_data = None
data_lock = threading.Lock()


serial_port = 'COM8'
baud_rate = 115200

Z_force = 0

# Serial Connection
ser = serial.Serial(serial_port, baud_rate, timeout=0.1)

def read_serial_data():
    global latest_serial_data
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line and line.startswith('X:'):
                line = line.replace(' uT', '').replace('\\', '').strip()
                parts = line.split()
                x = float(parts[1])
                y = float(parts[3])
                z = float(parts[5])
                with data_lock:
                    latest_serial_data = [x, y, z]
        except Exception as e:
            print(f"ERROR: {e}")
        time.sleep(0.001)



    

####################################################
# Move to the center of Workspace

def moveto_center():

    #if drd_dll.drdRegulatePos(True) < 0:
    #    print("Error: failed to set base regulation")
    #    return -1

    #if drd_dll.drdRegulateRot(True) < 0:
    #    print("Error: failed to set wrist regulation")
    #    return -1

    #if drd_dll.drdRegulateGrip(True) < 0:
    #    print("Error: failed to set gripper regulation")
    #    return -1


    if not drd_dll.drdIsInitialized() and drd_dll.drdAutoInit() < 0:
        print("error: failed to initialize device")
        time.sleep(2.0)
        sys.exit(-1)
    elif drd_dll.drdStart() < 0:
        print("error: failed to start robotic regulation")
        time.sleep(2.0)
        sys.exit(-1)

    positionCenter = (ctypes.c_double * 8)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


    if drd_dll.drdMoveTo(ctypes.byref(positionCenter), True, -1) < 0:
        print("error: failed to move the device")
        time.sleep(2)
        exit(-1)

    if drd_dll.drdStop(True) < 0:  # True is passed to indicate a forceful stop
        print(f"error: failed to stop robotic regulation ()")
        time.sleep(2.0)  # Sleep for 2 seconds
        sys.exit(-1)  # Exit the program with an error code

moveto_center()

moving = True

try: 
    
    haptic_thread = threading.Thread(target=haptic_input_thread)
    haptic_thread.start()

    serial_thread = threading.Thread(target=read_serial_data)
    serial_thread.start()

    # Main loop
    while running:

        if buffer:

            # Get Haptic input from Buffer
            haptic_input = buffer.popleft()


            with data_lock:
                serial_data = latest_serial_data
            if serial_data:
                x, y, z = serial_data
                
                # Prediction
                input_data = [[z]]  
                Z_predicted_force = model.predict(input_data)

                Z_force = Z_predicted_force[0][0] * 0.0098*0.5
                





            # Display current position
            current_time = time.time()
            if current_time - lastDisplayUpdateTime > 0.1:
                lastDisplayUpdateTime = current_time
                sys.stdout.write(f"\r Scale: {scale:.2f},Robot Max Range:{robot_max_range['x']:.3f} | Haptic Position: ({haptic_input[0]:6.3f} {haptic_input[1]:6.3f} {haptic_input[2]:6.3f} ")
                sys.stdout.flush()


            dz = (haptic_input[0] / haptic_max_range['x']) * robot_max_range['x']
            dy = (haptic_input[1] / haptic_max_range['y']) * robot_max_range['y']* -1
            dx = (haptic_input[2] / haptic_max_range['z']) * robot_max_range['z']
            new_position = [
                current_position[0] + dx,
                current_position[1] + dy,
                current_position[2] + dz
            ]
        
            new_pose = new_position + list(current_orientation)

             #Move to real time position
            if moving:
                robot.set_realtime_pose(new_pose)



        # Get current velocity (Dampling force)
        vel_result = drd_dll.drdGetVelocity(ctypes.byref(vx), ctypes.byref(vy), ctypes.byref(vz),
                                    ctypes.byref(wx), ctypes.byref(wy), ctypes.byref(wz),
                                    ctypes.byref(vg), ctypes.c_char(b'\xff') )
        if vel_result < 0:
            print("Failed to get velocity")
            time.sleep(2)
            break

        # Calculate forces
        fx = -damping_coefficient_translation * vx.value + Z_force
        fy = -damping_coefficient_translation * vy.value 
        fz = -damping_coefficient_translation * vz.value 
        tx = -damping_coefficient_rotation * wx.value
        ty = -damping_coefficient_rotation * wy.value
        tz = -damping_coefficient_rotation * wz.value

        # Apply the forces
        drd_dll.drdSetForceAndTorqueAndGripperForce(fx, fy, fz, tx, ty, tz, 0.0, ctypes.c_char(b'\xff') )


        if keyboard.is_pressed('x'):
            increase_scale()

        if keyboard.is_pressed('z'):
            decrease_scale()

        if keyboard.is_pressed('c'):
            print("Haptic device will move to center. ")
            moving = False
            moveto_center()
            current_pose = robot.get_actual_tcp_pose()  
            current_position = current_pose[:3]  
            time.sleep(0.1)
            moving = True

        if keyboard.is_pressed('r'):
             robot.movej(q=robot_startposition, a= 0.7, v= 0.8 )


        if keyboard.is_pressed('q'):
            running = False
            print("Exiting loop because 'q' was pressed.")
            # Close the connection to the haptic device
            if drd_dll.drdClose() < 0:
                print("error: failed to close the connection")
                time.sleep(2)
                sys.exit(-1)

            # Report success
            print("Haptic Connection Closed")
            robot_initialposition = (math.radians(0),
                            math.radians(-115),
                            math.radians(120),
                            math.radians(-180),
                            math.radians(-88),
                            math.radians(0))

            
            robot.movej(q=robot_initialposition, a= 0.7, v= 0.8 )
    
            
            print ("Robotic arms has move to initial state")
            print(" system ending")
            sys.exit(0)

            


except Exception as e:
    print("Stopping real-time control")

      
 
    # Close the connection to the haptic device
    if drd_dll.drdClose() < 0:
        print("error: failed to close the connection")
        time.sleep(2)
        sys.exit(-1)

    # Report success
    print("Haptic Connection Closed")
    robot_initialposition = (math.radians(0),
                    math.radians(-115),
                    math.radians(120),
                    math.radians(-180),
                    math.radians(-88),
                    math.radians(0))

    robot.movej(q=robot_initialposition, a= 0.7, v= 0.8 )
    
    print ("Robotic arms has move to initial state")
    sys.exit(0)



           




