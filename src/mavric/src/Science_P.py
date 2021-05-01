# -*- coding: utf-8 -*-
"""
Created on Sat May  1 14:18:20 2021

@author: Keith Harder
"""

import rospy
from std_msgs.msg import Float64


import time

import socket
from threading import *

data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

def talker():
    #Arm commands update one joint at a time, so we need one topic per joint
    pub_drill = rospy.Publisher("ScienceDrill", Float64, queue_size=10)
    
    pub_rotor = rospy.Publisher("ScienceRotor", Float64, queue_size=10)
    
    pub_lin_act = rospy.Publisher("ScienceLin", Float64, queue_size=10)
    
   
    
    rospy.init_node('ARP')
    port = rospy.get_param("port", 10002)
    serversocket.bind(('', port))
    serversocket.listen(1)

    rospy.loginfo('server started')

    while not rospy.is_shutdown():
        connection, address = serversocket.accept()
        data = connection.recv(1024).decode()

        if(data == ''):
            pass

        elif(data[0] == 'S'):
            # Arm Command
            parameters = data[2:].strip().split(',')
            rospy.loginfo(parameters)
            cmd = float(parameters[0])
            
            if data[1] == 'D':
                pub_drill.publish(cmd)
            
            elif data[1] == 'R':
                pub_rotor.publish(cmd)
            
            elif data[1] == 'L':
                pub_lin_act.publish(cmd)
            
        
            
        connection.close()
    serversocket.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
