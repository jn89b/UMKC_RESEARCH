#!/usr/bin/env python

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
from apriltag_ros.msg import AprilTagCoords
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
#Subscribe to center location of apriltag image
#Report back the coordinates based on the frame of the camera 
#Might want to try to get this info somewhere else later on 

###-----Initalize Variables------#
#Maximum x and y pose distances from camera -> Will need to adjust this min and max range
xmin = -6.25  
xmax = 4.75

#Camera specs we are running at 1280 x 720 will need to grab these values later on instead of hard coding this
W = 1280 
H = 720 

#total distance counting for negative values in x 
x_tot = xmax + abs(xmin)

#error vectors
er = [0,0]
#Desired positions on camera
cam_x_desired = 0 #((xmax+xmin)/2) * (W/xmax) #Could just use function scale

#Scale camera 
def scale(W,xmax,x):
    x_scale  = (W/xmax) * x
    return x_scale
   
class Target():
    def __init__(self,pub):
        self.AT_pub = pub
        #Minimium and maximum x ranges of pose from camera -> Need to change this 
        self.xmin = xmin
        self.xmax = xmax
        self.xtotal = xmax + abs(xmin)
        #Camera specs
        self.W = W
        self.H = H
        self.AT_topic_name = "/tag_detections" #Topic name for AprilTagdDetectionArray
        self.tag_detections_subscriber = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.check_AT)
        #Subscribing to sevomotors and  Apriltagdetection array to get pose positions
    
    def check_AT(self,msg):
        self.num_detections = len(msg.detections)
        if self.num_detections > 0:
            self.overall_pose = msg.detections[0].pose.pose.pose
            self.x = self.overall_pose.position.x
            #self.y = self.overall_pose.postion.y
            #Added 640 as an offset for the mid distance of the monitor width of 1280
            self.cam_x = scale(self.W,self.xtotal,self.x) + 640
            at = AprilTagCoords()
            at.x = self.cam_x
            rospy.loginfo('self.x:{}, self.cam_x={}'. format(self.x, self.cam_x))
            self.AT_pub.publish(at)
        else:
            print("Searching")

def main():
    rospy.init_node("apriltag_coord", anonymous=True)
    pub = rospy.Publisher("/AT_coords",AprilTagCoords,queue_size =10)
    Look_for = Target(pub)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

