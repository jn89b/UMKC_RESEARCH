#!/usr/bin/env python3

import rospy
import cv2
import time
#import serial
import numpy as np
#from apriltag import apriltag
#import tf
#import pi_servo_hat

from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
#Importing python script with class target
from apriltag_coord import Target

class PID():
    def __init__(self,kp,ki,kd,er):
       # self.kp = kp
       # self.ki = ki
       # self.kd = kd
       # self.er = er
       # #Don't worry about this just initializing the errors for the values 
       # self.P = 0 
       # self.I = 0 
       # self.D = 0 
       # self.error = 0 
        self.AT_data_processor = Target()

def control(er,kp,ki,kd,cam_x_desired,cam_x):
    while True():
        error = cam_x_desired - cam_x
        error_integral = eiold + ((er[1]+er[0])/dt) * dt
        error_der = (er[1] - er[0])/dt
        P = kp * error
        I = ki * error_integral
        D = kd * error_der
        PID = sum(P,I,D)
        return PID 

def start_apriltag_motor():
    rospy.init_node("motor_command", anonymous=True)
    pid = PID()
    rospy.spin()
if __name__ == '__main__':
    try:
        start_apriltag_motor()
    except rospy.ROSInterruptException:
        pass


