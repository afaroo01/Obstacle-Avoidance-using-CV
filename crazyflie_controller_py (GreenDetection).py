#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 
# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""

# Detects green floor and assumes obstacles if no green floor is found.
# Does not work properly


from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

import cv2
import numpy as np
import random
random.seed()

from math import cos, sin, atan2

import sys
sys.path.append('../../../controllers/')
from  pid_controller import init_pid_attitude_fixed_height_controller, pid_velocity_fixed_height_controller
from pid_controller import MotorPower_t, ActualState_t, GainsPID_t, DesiredState_t

robot = Robot()

timestep = int(robot.getBasicTimeStep())

## Initialize motors
m1_motor = robot.getDevice("m1_motor");
m1_motor.setPosition(float('inf'))
m1_motor.setVelocity(-1)
m2_motor = robot.getDevice("m2_motor");
m2_motor.setPosition(float('inf'))
m2_motor.setVelocity(1)
m3_motor = robot.getDevice("m3_motor");
m3_motor.setPosition(float('inf'))
m3_motor.setVelocity(-1)
m4_motor = robot.getDevice("m4_motor");
m4_motor.setPosition(float('inf'))
m4_motor.setVelocity(1)

## Initialize Sensors
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
Keyboard().enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
camera = robot.getDevice("camera")
camera.enable(timestep)
range_front = robot.getDevice("range_front")
range_front.enable(timestep)
range_left = robot.getDevice("range_left")
range_left.enable(timestep)
range_back = robot.getDevice("range_back")
range_back.enable(timestep)
range_right = robot.getDevice("range_right")
range_right.enable(timestep)
    
## Initialize variables
actualState = ActualState_t()
desiredState = DesiredState_t()
pastXGlobal = 0
pastYGlobal = 0
past_time = robot.getTime()

## Initialize PID gains.
gainsPID = GainsPID_t()
gainsPID.kp_att_y = 1
gainsPID.kd_att_y = 0.5
gainsPID.kp_att_rp =0.5
gainsPID.kd_att_rp = 0.1
gainsPID.kp_vel_xy = 2
gainsPID.kd_vel_xy = 0.5
gainsPID.kp_z = 10
gainsPID.ki_z = 50
gainsPID.kd_z = 5
init_pid_attitude_fixed_height_controller()


## Speeds
forward_speed = 0.4
yaw_rate = 0.8

## Avoidance state
avoid_yawDesired = 0
avoid_yawTime = 0

# Mouse callback function

def showHSV(event,x,y,flags,param):
    if event == cv2.EVENT_MOUSEMOVE:
        H = image_hsv[y, x, 0]
        S = image_hsv[y, x, 1]
        V = image_hsv[y, x, 2]
        label = f"{H}, {S}, {V}"
        cv2.putText(image_hsv, label, (x + 2, y - 5),
                    cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 0, 0))
        print(event)
        print("H, S, V = " + label)


## Initialize struct for motor power
motorPower = MotorPower_t()

print('Take off!')

# wayPoints = [(2, 3), (3, -1.5), (-3.5, 2)]
# idx = 0

# tol = 0.2
# reachFlag = False
# stopFlag = False

avoidStartFlag = False
avoidFlag = False

green_cx = 0
green_cy = 0

# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time;

    ## Get measurements
    actualState.roll = imu.getRollPitchYaw()[0]
    actualState.pitch = imu.getRollPitchYaw()[1]
    actualState.yaw_rate = gyro.getValues()[2];
    actualState.altitude = gps.getValues()[2];
    xGlobal = gps.getValues()[0]
    vxGlobal = (xGlobal - pastXGlobal)/dt
    yGlobal = gps.getValues()[1]
    vyGlobal = (yGlobal - pastYGlobal)/dt

    ## Get body fixed velocities
    actualYaw = imu.getRollPitchYaw()[2];
    cosyaw = cos(actualYaw)
    sinyaw = sin(actualYaw)
    actualState.vx = vxGlobal * cosyaw + vyGlobal * sinyaw
    actualState.vy = - vxGlobal * sinyaw + vyGlobal * cosyaw
    
    ## Initialize setpoints
    desiredState.roll = 0
    desiredState.pitch = 0
    desiredState.vx = 0
    desiredState.vy = 0
    desiredState.yaw_rate = 0
    desiredState.altitude = 1.5

    forwardDesired = forward_speed
    sidewaysDesired = 0
    yawDesired = 0

    ## Get camera image
    w, h = camera.getWidth(), camera.getHeight()
    cameraData = camera.getImage()  # Note: uint8 string
    image = np.fromstring(cameraData, np.uint8).reshape(h, w, 4)

    # Show image
    # cv2.imshow('Drone camera', image)
    # cv2.waitKey(1)

    ## Detect empty floor (green) in front of the drone
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    xmin, xmax = int(0.10 * w), int(0.90 * w)
    ymin, ymax = int(0.70 * h), int(1.00 * h)
    max_pix = (xmax - xmin) * (ymax - ymin)
    hmin, hmax = 30, 45
    roi_hue = image[ymin:ymax, xmin:xmax, 0]
    roi_sat = image[ymin:ymax, xmin:xmax, 1]
    green_binary_img = (roi_hue >= hmin) & (roi_hue <= hmax) & (roi_sat > 64)
    # green_binary_img = (roi_hue >= hmin) & (roi_hue <= hmax)
    green_binary_img = np.ndarray.astype(green_binary_img*255, dtype=np.uint8)

    # pix_count = np.count_nonzero((roi_hue >= hmin) & (roi_hue <= hmax) & (roi_sat > 64))
    pix_count = np.count_nonzero((roi_hue >= hmin) & (roi_hue <= hmax) & (roi_sat > 64))
    
    green_pct = pix_count / max_pix
    green_moments = cv2.moments(green_binary_img)
    
    if green_moments["m00"] >= 0.01:
        green_cx = xmin + int(green_moments["m10"]/green_moments["m00"])
        green_cy = ymin + int(green_moments["m01"]/green_moments["m00"])
    

    # print(f"Centroid_x = {green_cx}, Width = {w}")
    
    cv2.circle(image_hsv, (green_cx, green_cy), 5, (0, 0, 255), 2)
    centroid_label = f"{green_cx}, {green_cy}"
    cv2.putText(image_hsv, centroid_label, (green_cx + 6, green_cy - 12),
                cv2.FONT_HERSHEY_COMPLEX, 0.4, (0, 255, 0))
    
    cv2.imshow("Camera image (HSV)", image_hsv)
    cv2.setMouseCallback("Camera image (HSV)", showHSV)
    
    cv2.imshow("Green binary image", green_binary_img)
    
    cv2.waitKey(1)
    # if cv2.waitKey(1) == 27:
    #     # ASCII code of ESC = 27
    #     break
    
    ## Avoidance state machine
    if green_pct > 0.10:
        # print("State 1")
        # No obstacle: fly forwards
        forwardDesired = forward_speed
        if green_cx > w/2 + w/5:
            sign = -1
        elif green_cx < w/2 - w/5:
            sign = 1
        else:
            sign = 0
        
        yawDesired = sign*yaw_rate
        turn_rate = 0
        avoidFlag = False
        avoidStartFlag = False
    elif avoidStartFlag is False and avoidFlag is False:
        print("State 2")
        # Obstacle in front: need to start turn
        avoidStartFlag = True
        
    if avoidStartFlag is True:
        # Obstacle in front: start turn
        if green_cx > w/2:
            sign = -1
        else:
            sign = 1


        avoid_yawDesired = sign * yaw_rate
        yawDesired = avoid_yawDesired
        # forwardDesired = 0
        forwardDesired = -forward_speed/3
        avoidFlag = True
        avoidStartFlag = False
        
    elif avoidFlag is True:
        yawDesired = avoid_yawDesired
        forwardDesired = -forward_speed/3
            
    # if avoid_yawTime > 0:
    #     # Turning
    #     avoid_yawTime -= dt
    #     yawDesired += avoid_yawDesired
    # else:
    #     # Not turning
    #     if green_pct > 0.10:
    #         # No obstacle: fly forwards
    #         # forwardDesired += forward_speed
    #         forwardDesired = forward_speed
    #         sidewaysDesired = 0
    #         # yawDesired = yaw_rate
    #         yawDesired = 0
    #         turn_rate = 0
    #     else:
    #         # Obstacle in front: start turn
    #         sign = 1 if random.random() > 0.5 else -1
    #         avoid_yawDesired = sign * yaw_rate
    #         avoid_yawTime = random.random() * 5.0

    # # Manual override
    # key = Keyboard().getKey()
    # while key>0:
    #     if key == Keyboard.UP:
    #         forwardDesired = forward_speed
    #     elif key == Keyboard.DOWN:
    #         forwardDesired = -forward_speed
    #     elif key == Keyboard.RIGHT:
    #         sidewaysDesired  = -forward_speed
    #     elif key == Keyboard.LEFT:
    #         sidewaysDesired = forward_speed
    #     elif key == ord('Q'):
    #         yawDesired =  + yaw_rate
    #     elif key == ord('E'):
    #         yawDesired = - yaw_rate

    #     key = Keyboard().getKey()


    desiredState.yaw_rate = yawDesired
    desiredState.vx = forwardDesired
    desiredState.vy = sidewaysDesired


    ## PID velocity controller with fixed height
    # if idx == len(wayPoints):
    #     stopFlag = True
    # else:
    #     xTarget = wayPoints[idx][0]
    #     yTarget = wayPoints[idx][1]
    
    # if stopFlag is True:
    #     print("Stop flag activated")
    #     desiredState.vy = 0
    #     desiredState.vx = 0
    #     desiredState.yaw_rate = 0
    # elif abs(yTarget - yGlobal) <= tol and abs(xTarget - xGlobal) <= tol:
    #     idx += 1
    #     print(f"Index = {idx}")
    # else:
    #     theta = atan2(yTarget - yGlobal, xTarget - xGlobal)
    #     desiredState.vy = forward_speed*sin(theta)
    #     desiredState.vx = forward_speed*cos(theta)
        
    
    pid_velocity_fixed_height_controller(actualState, desiredState, gainsPID, dt, motorPower);

    m1_motor.setVelocity(-motorPower.m1)
    m2_motor.setVelocity(motorPower.m2)
    m3_motor.setVelocity(-motorPower.m3)
    m4_motor.setVelocity(motorPower.m4)
    
    past_time = robot.getTime()
    pastXGlobal = xGlobal
    pastYGlobal = yGlobal


cv2.destroyAllWindows()