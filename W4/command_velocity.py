#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities
            
wheel_radius = 0.35
robot_radius = 0.35
class spin():
	def __init__(self): #self. = this.
		rospy.loginfo("Starting info: ")
		self.wheel_sub = rospy.Subscriber("turtlebot_1/wheel_vel_left", Float32, self.callback)
		self.pub = rospy.Publisher("turtlebot_1/cmd_vel", Twist, queue_size = 1)

	def callback(self, data):
		(v, a) = self.forward_kinematics(data.data, 0.0)
		t = Twist()

		t.linear.x = v
		t.angular.z = a
		
		self.pub.publish(t)

		print "v = %f,\ta = %f" % (v, a)

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
	def inverse_kinematics(self, v, a):
		c_l = v + (robot_radius * a) / 2
		c_r = v - (robot_radius * a) / 2
		w_l = c_l / wheel_radius
		w_r = c_r / wheel_radius
		return (w_l, w_r)


	# inverse kinematics from a Twist message (This is what a ROS robot has to do)
	def inverse_kinematics_from_twist(self, t):
		return self.inverse_kinematics(t.linear.x, t.angular.z)

	#(w_l, w_r) = inverse_kinematics(0.0, 1.0)
	#print "w_l = %f,\tw_r = %f" % (w_l, w_r)

	#(v, a) = forward_kinematics(w_l, w_r)
	#print "v = %f,\ta = %f" % (v, a)


if __name__ == '__main__':
    rospy.init_node("command_velocity")
    spin()
    #cv = ()
    #cv.send_velocities() # Calling the function
    rospy.spin()