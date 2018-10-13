#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from mavric.msg import Autonomous, Waypoint

import socket
from threading import *

data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

enabled = False

host = ""
port = 9003

print(host)
print(port)

""" -MAIN LOOP- """
def talker():
    rospy.init_node("ANP")
    
    way_pub = rospy.Publisher("Next_Waypoint", Waypoint, queue_size=10)
    cmd_pub = rospy.Publisher("Autonomous", Autonomous, queue_size=10)

    serversocket.bind((host, port))
    serversocket.listen(1)

    while not rospy.is_shutdown():
        connection, address = serversocket.accept()
        data = connection.recv(1024).decode()

        rospy.loginfo(data)

        if data[0] == 'N' :
            if data[1] == 'E':
                #enable autonomous, disable regular drive publisher
                enabled = True
                cmd_pub.publish(enabled)

            elif data[1] == 'D':
                #disable autonomous, enable regular drive publisher
                enabled = False
                cmd_pub.publish(enabled)

            elif data[1] == 'W':
                #send waypoint to autonomous logic
                if enabled:
                    parameters = data[2:].strip().split(',')
                    rospy.loginfo(parameters)

                    way_pub.publish(float(parameters[0]), float(parameters[1]))

        connection.close()
    serversocket.close()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
