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

#Max range 
max_angle = 90
min_angle = 0

#PWM frequency 
pwm_freq = 50

# Initialize PWM frequency

tol = 10


#Servo motor command
class Motor_CMD():
    def __init__(self,servo):
        self.yaw_channel = yaw_channel
        #PID values
        self.kp = 1
        self.ki = 0
        self.kd = 0
        
        #Desired x position 
        self.x_desired = cam_x_desired
        self.tol = tol
        #Error rates for motor class
        self.er = er
        self.ei = ei
        self.ed = ed
        #Height and width of camera parameters
        self.W = W
        self.H = H
        
        #Motor and frequency
        self.servo = servo
        self.servo.set_pwm_frequency = servo.set_pwm_frequency
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.target_found = False
        #Wait for detection info topic to become avialble
        #rospy.wait_for_message("/tag_detections", AprilTagDetectionArray)
        self.rate = rospy.Rate(5)
        #Subscribe to Apriltag detections and ATcoordinates
        self.sub1 =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.check_AT)
        self.sub2 = rospy.Subscriber("/AT_coords", AprilTagCoords, self.somecallback) 
    #Check detections might code this better and store the value of the detections into AT_coords 
    def check_AT(self,msg):
        self.num_detections = len(msg.detections)
        self.search_AT
        if self.num_detections < 1:
            print("No AprilTag")
            #self.search_AT(servo=self.servo, min_angle=self.min_angle, max_angle=self.max_angle)
            pass 
        else:
            self.curr_yaw = self.servo.get_servo_position(self.min_angle,self.max_angle)
            self.Track(servo = self.servo, curr_yaw = self.curr_yaw)
            print(self.curr_yaw)

    def somecallback(self,msg):
        self.xcurr = msg.x
     
    def search_AT(self, servo, min_angle, max_angle):
        for i in range(min_angle,max_angle):
            servo.move_servo_position(min_angle,i,max_angle)
            time.sleep(0.1)
        for i in range(max_angle,min_angle,-1):
            servo.move_servo_position(min_angle,i,max_angle)
            time.sleep(0.1)
        
    def Track(self,servo,curr_yaw):
        self.PID_calc(er = self.er, ei=self.ei ,ed=self.ed ,kp=self.kp ,ki=self.ki,kd=self.kd, xdes=self.x_desired, xcurr=self.xcurr)
        if self.er_curr >= self.tol:
            #Move servo camera to the left or ccw
            print("going counter")
            curr_yaw = servo.move_servo_position(yaw_channel, curr_yaw + self.PID, max_angle) 
        elif self.er_curr <= self.tol:
            #Move servo camera to the right or cw 
            #curr_yaw = -.001 * curr_yaw
            curr_yaw = servo.move_servo_position(yaw_channel, curr_yaw - 1, max_angle) 
            print( curr_yaw)
        else: #er_curr<= tol:
            #Stay still
            print("We're good")
            pass
        
    def PID_calc(self,er,ei,ed,kp,ki,kd,xdes,xcurr):
        #Appending new values to old variables
        er[0] = er[1]
        eiold = ei
    
        #Error calculations
        er[1] = xdes - xcurr
        #print(er[1])
        er_curr = er[1]
        ei = eiold + ((er[1]+er[0])/2) * dt
        ed = (er[1] - er[0])/dt
    
        #PID calculations
        P = kp * er[1]
        I = ki * ei
        D = kd * ed 
    
        PID = (P+I+D)/100
        self.PID = PID
        self.er_curr = er_curr
        print(self.PID, self.er_curr) 
#def ret_PID():
#    return PID(self.PID,self.er_curr)
    
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
    #test.move_servo_position(0,max_angle, max_angle)

def main(): 
    #Servo parameters
    rospy.init_node('PanAndTilt', anonymous =True)
    servo = pi_servo_hat.PiServoHat()
    # Restart hat 
    servo.restart()
    servo.set_pwm_frequency(pwm_freq)
    ROM(servo)
    Tracking = Motor_CMD(servo)
    rospy.spin()
    
#Initializing script
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
