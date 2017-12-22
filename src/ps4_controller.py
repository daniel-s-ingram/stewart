#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

x = y = z = 0.0
roll = pitch = yaw = 0.0

def joy_callback(msg):
	global x, y, z, roll, pitch, yaw

	roll = -msg.axes[4]
	pitch = -msg.axes[3]
	if msg.buttons[5] and not msg.buttons[4]:
		yaw = yaw + 0.01
	elif msg.buttons[4] and not msg.buttons[5]:
		yaw = yaw - 0.01	

	x = -msg.axes[0]
	y = msg.axes[1]
	if msg.buttons[7] and not msg.buttons[6]:
		z = z + 0.01
	elif msg.buttons[6] and not msg.buttons[7]:
		z = z - 0.01

	if z > 1:
		z = 1
	elif z < 0:
		z = 0

	if yaw > 1.571:
		yaw = 1.571
	elif yaw < -1.571:
		yaw = -1.571

rospy.init_node('ps4_controller', anonymous=True)

joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)
twist_pub = rospy.Publisher('stewart/platform_twist', Twist, queue_size=100)

platform_twist = Twist()

while not rospy.is_shutdown():
	platform_twist.linear.x = x
	platform_twist.linear.y = y
	platform_twist.linear.z = z

	platform_twist.angular.x = roll
	platform_twist.angular.y = pitch
	platform_twist.angular.z = yaw

	twist_pub.publish(platform_twist)