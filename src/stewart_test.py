#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node('stewart_test', anonymous=True)
piston_pub = rospy.Publisher('/stewart/piston_cmd', Float32MultiArray, queue_size=10)

piston_lengths = Float32MultiArray()
piston_lengths.data = [1,1,1,0.5,0.5,1]

while not rospy.is_shutdown():
	piston_pub.publish(piston_lengths)
	piston_lengths.data = piston_lengths.data[-1:] + piston_lengths.data[0:-1]
	rospy.sleep(1.0)
