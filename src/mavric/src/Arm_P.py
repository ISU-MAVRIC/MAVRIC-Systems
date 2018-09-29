#!/usr/bin/env python
# Input node. Receives commands from the base station and publishes them in the
# arm ROS command format

import rospy
from std_msgs.msg import String
from mavric.msg import Arm

import time

import socket
from threading import *

data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = ""
port = 9002

print(host)
print(port)

def talker():
    #Arm commands update one joint at a time, so we need one topic per joint
    pub_shoulder_r = rospy.Publisher("Arm_ShoulderRot", Arm, queue_size=10)
    pub_shoulder_p = rospy.Publisher("Arm_ShoulderPitch", Arm, queue_size=10)
    
    pub_elbow_p = rospy.Publisher("Arm_ElbowPitch", Arm, queue_size=10)
    
    pub_wrist_r = rospy.Publisher("Arm_WristRot", Arm, queue_size=10)
    pub_wrist_p = rospy.Publisher("Arm_WristPitch", Arm, queue_size=10)
    
    pub_claw_a = rospy.Publisher("Arm_ClawActuation", Arm, queue_size=10)
    
    rospy.init_node('ARP')

    serversocket.bind((host, port))
    serversocket.listen(1)

    rospy.loginfo('server started')

    while not rospy.is_shutdown():
        connection, address = serversocket.accept()
        data = connection.recv(1024).decode()

        rospy.loginfo(data)
        cmd = float(parameters[0])

        if(data[0] == 'A'):
            # Arm Command
            parameters = data[2:].strip().split(',')
            rospy.loginfo(parameters)

            if data[1] == 'R':
                pub_shoulder_r.publish(cmd,0,0,0,0,0)

            elif data[1] == 'L':
                pub_shoulder_p.publish(0,cmd,0,0,0,0)

            elif data[1] == 'E':
                pub_elbow_p.publish(0,0,cmd,0,0,0)


        elif(data[0] == 'C'):
            # Claw Command
            parameters = data[2:].strip().split(',')
            rospy.loginfo(parameters)

            elif data[1] == 'R':
                pub_wrist_r.publish(0,0,0,cmd,0,0)

            if data[1] == 'P':
                pub_wrist_p.publish(0,0,0,0,cmd,0)

            elif data[1] == 'C':
                pub_claw_a.publish(0,0,0,0,0,cmd)

        connection.close()
    serversocket.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
