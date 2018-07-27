#!/usr/bin/env python

import rospy
from mavric.msg import GPS
import socket

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = ""
port = 9001
print (host)
print (port)

clients = [];

def callback(data):
        message = '%.10f,%.10f,%.3f,%.2f,%1f,%d\r\n' % (data.latitude, data.longititude, data.altitude, data.speed, data.heading, data.num_satellites)
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
	rospy.init_node('GPS_Interface')
        rospy.Subscriber("GPS_Data", GPS, callback, queue_size=10)
	serversocket.bind((host, port))
	serversocket.listen(1)
	rospy.loginfo('server started')
	while not rospy.is_shutdown():
		connection, address = serversocket.accept()
		clients.append(connection)
                print('new GPS Listener: ')
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

