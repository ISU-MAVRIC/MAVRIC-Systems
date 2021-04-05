#!/usr/bin/env python
"""
Created on Sat Apr  3 14:38:47 2021

@author: Keith Harder
"""

import socket
import rospy
from std_msgs.msg import String
from mavric.msg import Steer

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = ""
port = 9004
print(host)
print(port)

clients = []


def callback(data):
    lf = data.lf
    lb = data.lb
    rf = data.rf
    rb = data.rb

    message = '%d,%d,%d,%d\r\n' % (lf, lb, rf, rb)

    for client in clients:
        try:
            client.sendall(message.encode())

        except socket.error as e:
            print(e)
            client.close()
            clients.remove(client)
            print('client removed')
            print('%d clients remain\n' % len(clients))


def listener():

    rospy.init_node('listener', anonymous=True)
    port = rospy.get_param("~port", 9004)
    rospy.Subscriber("Steer_Feeback", Steer, callback, queue_size=10)

    serversocket.bind((host, port))
    serversocket.listen(1)
    rospy.loginfo('encoder socket connected')
    while not rospy.is_shutdown():
        connection, adress = serversocket.accept()
        clients.append(connection)
        print('we have a steer encoder listener: ')
        print(adress)
        print('\n')

    for client in clients:
        client.close()
    serversocket.close()


if __name__ == '__main__':
    listener()

'''steer_feedback'''
