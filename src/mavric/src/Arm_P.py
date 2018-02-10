#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import socket
from threading import *
data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = "127.0.0.1"
port = 9004
print (host)
print (port) #", line 228, in meth
#    return getattr(self._sock,name)(*args)
#<class '__main__.client'>", line 228, in meth
#    return getattr(self._sock,name)(*args)

#def soc(data):
#	print('ok')
#	serversocket.bind((host,port))
#	serversocket.listen(1)
#	connection, address = serversocket.accept()
#	data = connection.recv(1024).decode()

def arm1():
	pub = rospy.Publisher("/arm1", String, queue_size=10)
	rospy.init_node('AP1', anonymous=True)
	rate = rospy.Rate(10)
	serversocket.bind((host, port))
	serversocket.listen(1)
	rospy.loginfo('server started')
	while not rospy.is_shutdown():
		connection, address = serversocket.accept()
		data = connection.recv(1024).decode()
		pub.publish(data)
		print(data)
while __name__ == '__main__':
	try:
		arm1()
	except rospy.ROSInterruptException:
		pass

#		soc(data)
#		if data == '1':
#			arm1()
#		else:
#			pass
#	except rospy.ROSInterruptException:
#		pass
