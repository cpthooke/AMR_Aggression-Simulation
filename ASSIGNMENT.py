#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy #Allows for python.
import cv2 #Allows for OpenCV.
import numpy #Allows for python mathematics.
from sensor_msgs.msg import Image #Imports ability to read images.
from cv_bridge import CvBridge, CvBridgeError #Allows for conversion.
from std_msgs.msg import Float32 #Allows for ROS messages/data types.
from geometry_msgs.msg import Twist #Imports angular posing.

            #Simulator: roslaunch uol_turtlebot_simulator labc.launch
            #Rviz: roslaunch uol_turtlebot_simulator view_navigation.launch     
            #Keyboard input: roslaunch uol_turtlebot_simulator keyop.launch robot_name:=turtlebot_2
            
wheel_radius = 0.35
robot_radius = 0.35

class Assignment():
    def __init__(self): #self. = this.
        rospy.loginfo("Starting info: ")
        cv2.namedWindow("Left Window", 1) #Defines the two window halves (left & right).
        cv2.namedWindow("Right Window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge() #Makes a bridge that converts ROS images to OpenCV images.
		
#########################For the REAL ROBOT#########################								
        #self.wheel_sub = rospy.Subscriber("/wheel_vel", Float32, self.callback) #Subscribes to the wheel velocity of the robot (reads any information as it is published from the requested source).
        #self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) #Publishes requested information to method.
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
        #                                  Image, self.callback) #Subscribes to the image data from the robot.
#########################For the SIMULATION#########################								
        self.wheel_sub = rospy.Subscriber("turtlebot_1/wheel_vel", Float32, self.callback) #Following lines are similar to those above but are based around the Turtlebot simulation.
        self.pub = rospy.Publisher("turtlebot_1/cmd_vel", Twist, queue_size = 1)
        self.image_sub = rospy.Subscriber("turtlebot_1/camera/rgb/image_raw",
                                          Image, self.callback)
    def callback(self, data):						
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        image_size = cv_image.shape #Defines image output size dependent on window size (defined below).
        print image_size[0]
        print image_size[1]
      
        l = cv_image[:, 0:320] #Defines window size of left and right hand window.
        r = cv_image[:, 320:640]				
        
#########################LEFT Hand Window#########################								
	####################BGR Model####################								
        #bgr_thresh_l = cv2.inRange(l,
        #                        numpy.array((0, 0, 0)),
        #                        numpy.array((0, 255, 0))) #Searches for any green colour
        #
        
        #bgr_contours_l, hierachy_l = cv2.findContours(bgr_thresh_l.copy(),
        #                        cv2.RETR_TREE,
        #                        cv2.CHAIN_APPROX_SIMPLE) #Defines the areas that are contained in the search threshold.
		
	####################HSV Model####################								
        hsv_img_l = cv2.cvtColor(l, cv2.COLOR_BGR2HSV) #Converts image colour to hue/saturation/value(lightness) colour model.
		
        hsv_thresh_l = cv2.inRange(hsv_img_l,
                                numpy.array((60, 200, 0)),
                                numpy.array((130, 255, 255))) #Searches for any green colour.
                 
        hsv_contours_l, hierachy_l = cv2.findContours(hsv_thresh_l.copy(),
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE) #Defines the areas that are contained in the search threshold.
        for c in hsv_contours_l:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(l, c, -1, (255, 0, 0),3) #Draws dots of the colours found in the search threshold.
                speed = 0.4
                t = Twist()
                t.linear.x, t.angular.z = self.forward_kinematics(speed, speed + 0.2) #Increases the speed of the right wheel when a contour is found in the left hand side.
                self.pub.publish(t) #Publishes the information.
                
#########################RIGHT Hand Window#########################
	####################BGR Model####################								
        #bgr_thresh_r = cv2.inRange(r,
        #                        numpy.array((0, 0, 0)),
        #                        numpy.array((0, 255, 0)))
        
        #bgr_contours_r, hierachy_r = cv2.findContours(bgr_thresh_r.copy(),
        #                        cv2.RETR_TREE,
        #                        cv2.CHAIN_APPROX_SIMPLE)

	####################HSV Model####################								
        hsv_img_r = cv2.cvtColor(r, cv2.COLOR_BGR2HSV)
        
        hsv_thresh_r = cv2.inRange(hsv_img_r,
                                numpy.array((60, 200, 0)),
                                numpy.array((130, 255, 255)))
                 
        hsv_contours_r, hierachy_r = cv2.findContours(hsv_thresh_r.copy(),
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)

        for c in hsv_contours_r:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(r, c, -1, (255, 0, 0),3)
                speed = 0.4
                t = Twist()
                t.linear.x, t.angular.z = self.forward_kinematics(speed + 0.2, speed) #Increases the speed of the left wheel when a contour is found in the right hand side.
                self.pub.publish(t)
        
        print '===='
        cv2.imshow("Left Window", l)
        cv2.imshow("Right Window", r) #Opens the two window halves.
								

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber()
        rospy.Spin()
		
    def forward_kinematics(self, w_r, w_l): #Moves the robot.
        c_l = wheel_radius * w_l
        c_r = wheel_radius * w_r
        v = (c_l + c_r) / 2 #Defines the linear velocity.
        a = (c_l - c_r) / robot_radius #Defines the angular velocity
        return (v, a)

	#def inverse_kinematics_from_twist(self, t):
     #   return self.inverse_kinematics(t.linear.x, t.angular.z)


if __name__ == '__main__':
    rospy.init_node("ASSIGNMENT") #Initialises the program.
    this = Assignment() #Initialises the class.
    rospy.spin()
    cv2.destroyAllWindows() #Closes all windows upon exit.