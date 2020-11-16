# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 15:53:25 2020

@author: jnguy
"""

#Import stuff

import pi_servo_hat
import time
import sys
from apriltag_ros.msg import Servo

class PanandTiltmove(objet):
    
    def __init__(self):
        
        #Set object of the piservophat communicator
        self._pwm = pi_servo_hat.pi_servo_hat()
        
        #Soft resets the PCA and returns the PWM frequency to 50HZ
        self._pwm = restart()
        
        #Frequency of the PWM 
        self._PWM_FREQ = 50 

        #Set frequency from above 
        self._pwm.set_pwm_frequency(self._PWM_FREQ)
        
        #Set initial heading/yaw angle to 0 
        self.YAW_INIT_ANGLE = 0
        
        #Channels of yaw camera
        self.yaw_channel = 0
        
        #Calls the move_to_yaw function 
        self.move_to_yaw(yaw=self.YAW_INIT_ANGLE)
        
        #Move pan/yaw servo
        def move_to_yaw(self,yaw):
            self.move_yaw(yaw_angle=yaw)
        
        #Rotate the yaw angle
        def move_yaw(self,yaw_angle):
            self._pwm.setRotationAngle(self.yaw_channel, yaw_angle)
        
        #This method tests the full range of the yaw servo motor once
        def yaw_range_test(self): 
            for angle in range(10,80,1):
                print("Moving Yaw=" +str(angle))
                self.move_yaw(yaw_angle=angle)
                time.sleep(0.1)
                
            for angle in range(10,80,-1):
                print("Moving Yaw=" +str(angle))
                self.move_yaw(yaw_angle=angle)
                time.sleep(0.1)
            
        #This method/function allows user to input an angle for the user to enter
        #Utilizes the move_to_yaw function 
        def input_pitch_test(self):
            
            while True:
                input_x = raw_input("Type Angle to move")
                angle = int(input_x)
                print("Moving Yaw" + str(angle))
                self.move_to_yaw(yaw_angle=angle)

             