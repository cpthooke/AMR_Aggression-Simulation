#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities
            
wheel_radius = 0.35
robot_radius = 0.35

class Assignment():
    def __init__(self): #self. = this.
        rospy.loginfo("Starting info: ")
        cv2.namedWindow("Left Window", 1)
        cv2.namedWindow("Right Window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()        
        self.wheel_sub = rospy.Subscriber("turtlebot_1/wheel_vel_left", Float32, self.callback)
        self.pub = rospy.Publisher("turtlebot_1/cmd_vel", Twist, queue_size = 1)
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",
                                          Image, self.callback)

    def callback(self, data):
        #(v, a) = self.forward_kinematics(data.data, 0.0)
        #t = Twist()
             
        #t.linear.x = v
        #t.angular.z = a
             
        #self.pub.publish(t)
								
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        image_size = cv_image.shape
        print image_size[0]
        print image_size[1]
      
        #b, g, r = cv2.split(cv_image)
        l = cv_image[:, 0:320]
        r = cv_image[:, 320:640]				
        #px = img[100,100]
        #print px #[157 166 200]
								
        bgr_thresh = cv2.inRange(cv_image,
                                numpy.array((0, 0, 0)),
                                numpy.array((0, 255, 0)))
        
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        hsv_thresh = cv2.inRange(hsv_img,
                                numpy.array((60, 200, 0)),
                                numpy.array((130, 255, 255)))
                 
        bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(),
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)

        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)

        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0),3)
        
        #IplImage* frame1 = 
        
        print '===='
        cv2.imshow("Left Window", l)
        cv2.imshow("Right Window", r)
								

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber()
        rospy.Spin()
		
    # computing the forward kinematics for a differential drive
    def forward_kinematics(self, w_r, w_l):
        c_l = wheel_radius * w_l
        c_r = wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_l - c_r) / robot_radius
        return (v, a)


	# computing the inverse kinematics for a differential drive
	#def inverse_kinematics(self, v, a):
	#	c_l = v + (robot_radius * a) / 2
	#	c_r = v - (robot_radius * a) / 2
	#	w_l = c_l / wheel_radius
	#	w_r = c_r / wheel_radius
	#	return (w_l, w_r)


	# inverse kinematics from a Twist message (This is what a ROS robot has to do)
    def inverse_kinematics_from_twist(self, t):
        return self.inverse_kinematics(t.linear.x, t.angular.z)

	#(w_l, w_r) = inverse_kinematics(0.0, 1.0)
	#print "w_l = %f,\tw_r = %f" % (w_l, w_r)

	#(v, a) = forward_kinematics(w_l, w_r)
	#print "v = %f,\ta = %f" % (v, a)


if __name__ == '__main__':
    rospy.init_node("ASSIGNMENT")
    this = Assignment()
    #cv = ()
    #cv.send_velocities() # Calling the function
    rospy.spin()
    cv2.destroyAllWindows()