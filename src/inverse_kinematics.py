#!/usr/bin/env python
import rospy
import numpy as np
from numpy import cos, sin
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

h = 2.0 #Height of platform above base when legs are completely unextended

b = np.array([[-0.101, 0.8, 0.25],
              [0.101, 0.8, 0.25],
              [0.743, -0.313, 0.25],
              [0.642, -0.487, 0.25],
              [-0.643, -0.486, 0.25],
              [-0.744, -0.311, 0.25]])

p = np.array([[-0.642, 0.487, -0.05],
              [0.642, 0.487, -0.05],
              [0.743, 0.313, -0.05],
              [0.101, -0.8, -0.05],
              [-0.101, -0.8, -0.05],
              [-0.743, 0.313, -0.05]])

def twist_callback(twist_msg):
    x = twist_msg.linear.x
    y = twist_msg.linear.y
    z = twist_msg.linear.z
    roll = twist_msg.angular.x
    pitch = twist_msg.angular.y
    yaw = twist_msg.angular.z
    for i in range(n_pistons):
        l = np.array([x, y, z + h]) + np.dot(rotation_matrix(roll, pitch, yaw), p[i]) - b[i]
        piston_lengths.data[i] = np.sqrt(l[0]**2 + l[1]**2 + l[2]**2)-h

    piston_pub.publish(piston_lengths)

def rotation_matrix(rho, theta, psi):
    return np.array([[cos(psi)*cos(theta), -sin(psi)*cos(rho)+cos(psi)*sin(theta)*sin(rho), sin(psi)*sin(rho)+cos(psi)*sin(theta)*cos(rho)],
                     [sin(psi)*cos(theta), cos(psi)*cos(rho)+sin(psi)*sin(theta)*sin(rho), -cos(psi)*sin(rho)+sin(psi)*sin(theta)*cos(rho)],
                     [-sin(theta), cos(theta)*sin(rho), cos(theta)*cos(psi)]])

rospy.init_node('ik')
twist_sub = rospy.Subscriber('/stewart/platform_twist', Twist, twist_callback)
piston_pub = rospy.Publisher('/stewart/piston_cmd', Float32MultiArray, queue_size=10)
piston_lengths = Float32MultiArray()
piston_lengths.data = [0, 0, 0, 0, 0, 0]
n_pistons = len(piston_lengths.data)
length_components = [0, 0, 0]
rospy.spin()