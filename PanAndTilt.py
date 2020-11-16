#!/usr/bin/env python3

import rospy
#import cv2
import time
import numpy as np
#import tf
import pi_servo_hat
#Importing python script with class target
from apriltag_coord import Target

from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from apriltag_ros.msg import AprilTagCoords
from apriltag_ros.msg import Servo
from apriltag_ros.msg import RC
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

#Set minimum and maximum magnitude
max_val = 100
min_val = 0

#Refresh rate
refresh_rate = 50

#Dumb variable
dummy_var = 1

#Servo throttle channel
yaw_channel = 0

#Initializing classes from messages
#cmd = Servo()
#rc = RC()

#CAMERA PARAMETERS ---------------------------------------------------------------------- 
#Maximum x and y pose distances from camera -> Will need to adjust this min and max range
xmin = -6.25  
xmax = 4.75

#Camera specs we are running at 1280 x 720 will need to grab these values later on instead of hard coding this
W = 1280 
H = 720 

#total distance counting for negative values in x 
x_tot = xmax + abs(xmin)

#PID PARAMETERS ---------------------------------------------------------------------------
#Desired positions on camera and tolerance accepted
cam_x_desired = 640 #((xmax+xmin)/2) * (W/xmax) #Could just use function scale
tol = 10 # Might change to a certain perctange
kp = 1 # Will need to tune these parameters later on
ki = 0
kd = 0

#initial error values
er = [0,0] #Error readings for Proportional
ei = 0 # Integral Error 
ed = 0 # Derivative error

dt = 0.1

#INITIALIZE SERVO MOTOR-----------------------------------------------------------------------
# Initialize Constructor and servo 
servo = pi_servo_hat.PiServoHat()

# Restart hat 
servo.restart()

#Max range 
max_angle = 90
min_angle = 0

#PWM frequency 
pwm_freq = 50

# Initialize PWM frequency
servo.set_pwm_frequency(pwm_freq)
tol = 10

#Callback functions 
def somecall(msg):
    global num_detections
    num_detections = len(msg.detections)

def somecallback(msg):
    global x_at  
    x_at = msg.x

#Look for the AT between the maximum and minimum angle parameters
def AT_Look(test,max_angle,min_angle):
    for i in range(min_angle, max_angle):
        print("Input: ", end = '')
        print(i, end = '')
        test.move_servo_position(min_angle, i, max_angle)
        print(" Estimated Pos: ", end = '')
        print(test.get_servo_position(min_angle, max_angle))
        time.sleep(.1)
    for i in range(max_angle, min_angle, -1):
        print("Input: ", end = '')
        print(i, end = '')
        test.move_servo_position(min_angle, i, max_angle)
        print(" Estimated Pos: ", end = '')
        print(test.get_servo_position(min_angle, max_angle))
        time.sleep(.1)

#Initialize and Check full range of motion of ROM
def ROM(test):
    # Test Run
    #########################################
    # Moves servo position to 0 degrees (1ms), Channel 0
    test.move_servo_position(0, min_angle, max_angle)

    # Pause 1 sec
    time.sleep(1)

    # Moves servo position to 180 degrees (2ms), Channel 0
    test.move_servo_position(0,max_angle, max_angle)

def PID(er,ei,ed,kp,ki,kd,xdes,xcurr):
    #Appending new values to old variables
    er[0] = er[1]
    eiold = ei

    #Error calculations
    er[1] = xdes - xcurr
    print(er[1])
    er_curr = er[1]
    ei = eiold + ((er[1]+er[0])/2) * dt
    ed = (er[1] - er[0])/dt

    #PID calculations
    P = kp * er[1]
    I = ki * ei
    D = kd * ed 

    PID = (sum(P,I,D))/100
    return(PID,er_curr)

#Keep looking for AT 
def Track():
    global x_at, num_detections
        #If we find the apriltag center... we will implement the PID to keep track of it
    num_detections 
    if num_detections > 0:
    #Start calculating them errors
        #Current heading position of camera between the min and max angles
        curr_yaw = servo.get_servo_position(min_angle,max_angle)
        PID(er,ei,ed,kp,ki,kd,cam_x_des,x_at)
        print(PID)
        if er_curr > 0:
            #Move servo camera to the left or ccw
            curr_yaw = servo.move_servo_position(yaw_channel, curr_yaw + PID, max_angle) 
        else: #er_curr < 0: 
            #Move servo camera to the right or cw 
            curr_yaw = servo.move_servo_position(yaw_channel, curr_yaw - PID, max_angle) 
        #else er_curr<= tol:
            #Stay still
           # pass
    else:
        AT_Look(servo,max_angle,min_angle)

    rospy.spin()

#Initializing script
if __name__ == "__main__":
    rospy.init_node('PanAndTilt', anonymous =True)
    ROM(servo)
    #Subscribe to the center of apriltags
    #Subscribe to apriltag detections 
    sub1 = rospy.Subscriber("/tag_detections",AprilTagDetectionArray, somecall)
    sub2 = rospy.Subscriber("/AT_coords", AprilTagCoords, somecallback)
    print(num_detections)
    Track()





