#!/usr/bin/env python

import rospy
from mavric.msg import Auto_Status
from std_msgs.msg import Bool
import socket

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = ""
port = 11001

clients = []

def send_socket_message(message):
    for client in clients:
        try:
            client.sendall(message.encode())
        except socket.error as e:
            print(e)
            client.close()
            clients.remove(client)

def data_callback(data):
    message = '%d,%d,%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f' % (data.enable, data.hasGPS, data.hasIMU, data.waypoints, data.tgtLon, data.tgtLat. data.desiredHeading, data.deltaHeading, data.linearError)
    send_socket_message(message)

def talker():
    rospy.init_node('Auto_Interface')
    rospy.Subscriber("Status", Auto_Status, data_callback, queue_size=10)
    serversocket.bind((host, port))
    serversocket.listen(1)
    rospy.loginfo('server started')
    while not rospy.is_shutdown():
        connection, address = serversocket.accept()
        clients.append(connection)
        #print('new GPS Listener: ')
        #print(address)
        #print('\n')
                
        for client in clients:
            client.close()

    serversocket.close()	
	

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass