#!/usr/bin/env python
# Reads the Temperature topic and streams it to any connected clients as text.
# Somply connect to the port to recieve data

import rospy
from std_msgs.msg import Float64
import socket

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = ""
port = 8002
print (host)
print (port)

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
	rospy.init_node('Temperature_Interface')
        rospy.Subscriber("Temperature", Float64, callback, queue_size=10)
	serversocket.bind((host, port))
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

