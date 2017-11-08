#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import socket
from threading import *
data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = "127.0.0.1"
port = 9001
print (host)
print (port)

print('where ros starts')
#<class '__main__.client'>
def talker():
	pub = rospy.Publisher("/chatter", String, queue_size=10)
	rospy.init_node('talker0', anonymous=True)
	rate = rospy.Rate(10)
	serversocket.bind((host, port))
	serversocket.listen(1)
	rospy.loginfo('server started')
	while not rospy.is_shutdown():
		
		connection, address = serversocket.accept()
		data = connection.recv(1024).decode()
		connection.close()
		pub.publish(data)
		rate.sleep()
	serversocket.close()	
	

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

