#!/usr/bin/env python
# Reads the Float64 topic and streams it to any connected clients as text.
# Simply connect to the port to recieve data

# Topics:
#   value - Subscription: Listens for updates and publishes them to all connected clients.

import rospy
from std_msgs.msg import Float64
import socket

clients = [];

def callback(data):
        message = '%.3f\r\n' % (data.data)
        for client in clients:
                try:
                        client.sendall(message.encode())
                except socket.error as e:
                        print(e)
                        client.close();
                        clients.remove(client)
                        print('client removed\n')
                        print('%d clients remain\n' % len(clients))

def talker():
	rospy.init_node('Float64_Interface_Out')
        rospy.Subscriber("value", Float64, callback, queue_size=10)
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        port = rospy.get_param("~port", -1)
        if port == -1:
                raise ValueError("port not set")
	serversocket.bind(("", port))
	serversocket.listen(1)
	rospy.loginfo('server started')
	while not rospy.is_shutdown():
		connection, address = serversocket.accept()
		clients.append(connection)
                print('new Temperature Listener: ')
                print(address)
                print('\n')
                
        for client in clients:
                client.close()
	serversocket.close()	
	

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

