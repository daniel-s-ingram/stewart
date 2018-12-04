#!/usr/bin/env python
from math import pi
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

platform_twist = Twist()

def joy_callback(msg):
	platform_twist.linear.x = -msg.axes[0]
	platform_twist.linear.y = msg.axes[1]
	platform_twist.angular.x = -msg.axes[4]
	platform_twist.angular.y = -msg.axes[3]
	if msg.buttons[7] and platform_twist.linear.z < 1:
		platform_twist.linear.z += 0.01
	elif msg.buttons[6] and platform_twist.linear.z > 0:
		platform_twist.linear.z -= 0.01
	if msg.buttons[5] and platform_twist.angular.z < pi/2:
		platform_twist.angular.z += 0.01
	elif msg.buttons[4] and platform_twist.angular.z > -pi/2:
		platform_twist.angular.z -= 0.01	

	twist_pub.publish(platform_twist)

rospy.init_node('ps4_controller')
joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)
twist_pub = rospy.Publisher('stewart/platform_twist', Twist, queue_size=10)
rospy.spin()
