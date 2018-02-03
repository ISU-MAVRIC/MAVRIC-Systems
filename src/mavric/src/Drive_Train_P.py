#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import socket
from threading import *
data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = "10.26.196.148"
port = 9002
print (host)
print (port)#", line 228, in meth
#    return getattr(self._sock,name)(*args)
#<class '__main__.client'>", line 228, in meth
#    return getattr(self._sock,name)(*args)

def talker():
	pub = rospy.Publisher("/Drive_Train", String, queue_size=10)
	rospy.init_node('DTP', anonymous=True)
	rate = rospy.Rate(10)
	serversocket.bind((host, port))
	serversocket.listen(1)
	rospy.loginfo('server started')
	while not rospy.is_shutdown():
		connection, address = serversocket.accept()
		data = connection.recv(1024).decode()

		print(data)
		connection.close()
		pub.publish(data)
		rate.sleep()
	serversocket.close()	
	

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

