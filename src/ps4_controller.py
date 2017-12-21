#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

roll = pitch = 0.0

def joy_callback(msg):
	global roll, pitch

	roll = msg.axes[4]
	pitch = msg.axes[3]

rospy.init_node('ps4_controller', anonymous=True)

joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)
twist_pub = rospy.Publisher('stewart/platform_twist', Twist, queue_size=20)

platform_twist = Twist()
platform_twist.linear.x = 0.0
platform_twist.linear.y = 0.0
platform_twist.linear.z = 0.0
platform_twist.angular.x = 0.0
platform_twist.angular.y = 0.0
platform_twist.angular.z = 0.0

while not rospy.is_shutdown():
	platform_twist.angular.x = roll
	platform_twist.angular.y = pitch

	twist_pub.publish(platform_twist)