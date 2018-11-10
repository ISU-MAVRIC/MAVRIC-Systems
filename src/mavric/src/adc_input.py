#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
	pub=rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker')
	rate=rospy.Rate(10) # 10
	while not rospy.is_shutdown():
		#V21=read_from ADC(...)
		Rospy.loginfo(V21)
		Pub.publish(V21)
		Rate.sleep()

def read_from_ADC():
	pub=rospy.Publisher(...)
	rospy.init_node(...)
	rate=rospy.Rate(..)# 100
	while not rospy.is_shutdown():
		Subscribe<std_msgs:strings("V21",...,chatter Callback);
