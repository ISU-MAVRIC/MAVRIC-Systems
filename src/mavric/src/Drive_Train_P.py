#!/usr/bin/env python
# Input node. Recieves commands from the base station and publishes them as ros messages to the Drive_Train, Steer_Train, Steer_Cal topic.
# command format: D<left>,<right> left and right are floating-point numbers.
# command format: S<left front>,<left back>,<right front>,<right back> all are floating-point numbers.
# command format: C<left front>,<left back>,<right front>,<right back> all are floating-point numbers.

# Topics:
#   Drive_Train - Publication: publishes user drive commands to ROS.
#   Steer_Train - Publication: publishes user steer commands to ROS.
#   Steer_Cal - Publication: publishes steer position calibration commands to ROS.

import rospy
from std_msgs.msg import String
from mavric.msg import Drivetrain
from mavric.msg import Steertrain
from mavric.msg import Steercal

import time

import socket
from threading import *
data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

enabled = True

host = ""
port = 9002

# print(host)
# print(port)  # ", line 228, in meth
#    return getattr(self._sock,name)(*args)
# <class '__main__.client'>", line 228, in meth
#    return getattr(self._sock,name)(*args)


def talker():
    global enabled

    drive = rospy.Publisher("Drive_Train", Drivetrain, queue_size=10)
    steer = rospy.Publisher("Steer_Train", Steertrain, queue_size=10)
    cal = rospy.Publisher("Steer_Cal", Steercal, queue_size=10)
    rospy.init_node('DTP')
    port = rospy.get_param("~port", 9002)
    # print(port)
    serversocket.bind(('', port))
    serversocket.listen(1)
    #rospy.loginfo('server started')
    while not rospy.is_shutdown():
        connection, address = serversocket.accept()
        data = connection.recv(1024).decode()
        # rospy.loginfo(data)
        if (data[0] == 'D'):
            # Drive Command
            parameters = data[1:].strip().split(',')
            # rospy.loginfo(parameters)

            if enabled:		
                drive.publish(float(parameters[0]), float(parameters[1]), float(parameters[2]), float(parameters[3]), float(parameters[4]), float(parameters[5]))
                steer.publish(float(parameters[6]), float(parameters[7]), float(parameters[8]), float(parameters[9]))

        elif (data[0] == 'S'):
            # Steer Command
            parameters = data[1:].strip().split(',')
            # rospy.loginfo(parameters)

            if enabled:
                strlf = float(parameters[0])
                strlb = float(parameters[1])
                strrf = float(parameters[2])
                strrb = float(parameters[3])
                steer.publish(strlf, strlb, strrf, strrb)

        elif (data[0] == 'C'):
            # Cal Command
            parameters = data[1:].strip().split(',')
            # rospy.loginfo(parameters)

            if enabled:
                callf = float(parameters[0])
                callb = float(parameters[1])
                calrf = float(parameters[2])
                calrb = float(parameters[3])
                cal.publish(callf, callb, calrf, calrb)

        elif (data[0] == 'N'):
            # Autonomous Command
            if data[1] == 'E':
                # enable autonomous, disable drive
                enabled = False

            elif data[1] == 'D':
                # disable autonomous, enable drive
                enabled = True

        connection.close()
    serversocket.close()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
