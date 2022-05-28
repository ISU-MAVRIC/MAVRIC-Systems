#!/usr/bin/env python
# Input node. Receives commands from the base station and publishes them in the
# arm ROS command format
#
# Topics:
#   ShoulderRot - The desired motion of the shoulder rotation motor
#   ShoulderPitch - The desired motion of the shoulder pitch motor
#   ElbowPitch - The desired motion of the elbow pitch motor
#   WristRot - The desired motion of the shoulder rotation motor
#   WristRot - The desired motion of the wrist rotation motor
#   WristPitch - The desired motion of the wrist pitch motor
# The ranges for the signals is -100,100 coresponding to the motor controllers' extremum.

import rospy
from std_msgs.msg import Float64, Bool
from mavric.msg import Arm
from geometry_msgs.msg import Twist

import time

import socket
from threading import *

data = 'stop'
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

def talker():
    global enabled
    enabled = 0
    timer = time.time()
    #Arm commands update one joint at a time, so we need one topic per joint
    pub_shoulder_r = rospy.Publisher("ShoulderRot", Float64, queue_size=10)
    pub_shoulder_p = rospy.Publisher("ShoulderPitch", Float64, queue_size=10)
    
    pub_elbow_p = rospy.Publisher("ElbowPitch", Float64, queue_size=10)
    
    pub_wrist_r = rospy.Publisher("WristRot", Float64, queue_size=10)
    pub_wrist_p = rospy.Publisher("WristPitch", Float64, queue_size=10)
    
    pub_claw_a = rospy.Publisher("ClawActuation", Float64, queue_size=10)
    pub_claw_pos = rospy.Publisher("ClawPosition", Twist, queue_size=10)

    pub_arm_enable = rospy.Publisher("ArmEnable", Bool, queue_size=10)

    pub_hook_a = rospy.Publisher("HookActuation", Float64, queue_size=10)

    pub_cam_rot = rospy.Publisher("CamRot", Float64, queue_size=10)
    pub_cam_Pitch = rospy.Publisher("CamPitch", Float64, queue_size=10)

    pub_util = rospy.Publisher("Util", Float64, queue_size=10)
    pub_sci = rospy.Publisher("Sci", Float64, queue_size=10)

    
    
    rospy.init_node('ARP')
    port = rospy.get_param("port", 10001)
    serversocket.bind(('', port))
    serversocket.listen(1)

    #rospy.loginfo('server started')

    while not rospy.is_shutdown():
        connection, address = serversocket.accept()
        data = connection.recv(1024).decode()

        if(data == ''):
            pass

        elif(data[0] == 'A') and enabled is not True:
            # Arm Command
            parameters = data[2:].strip().split(',')
            #rospy.loginfo(parameters)
            cmd = float(parameters[0])
            
            if data[1] == 'R':
                pub_shoulder_r.publish(cmd)
            
            elif data[1] == 'L':
                pub_shoulder_p.publish(cmd)
            
            elif data[1] == 'E':
                pub_elbow_p.publish(cmd)
            
            
        elif(data[0] == 'C') and enabled is not True:
            # Claw Command
            parameters = data[2:].strip().split(',')
            #rospy.loginfo(parameters)
            cmd = float(parameters[0])
            
            if data[1] == 'R':
                pub_wrist_r.publish(cmd)
            
            elif data[1] == 'P':
                pub_wrist_p.publish(cmd)
            
            elif data[1] == 'C':
                pub_claw_a.publish(cmd)
        
        
        elif(data[0] == 'P') and enabled is True:
            # Claw Pos
            if time.time()-timer >= 0.1:
                parameters = data[1:].strip().split(',')
                cmd = list(map(float, parameters))
                values = Twist()
                values.linear.x = cmd[0]
                values.linear.y = cmd[1]
                values.linear.z = cmd[2]
                values.angular.x = cmd[3]
                values.angular.y = cmd[4]
                values.angular.z = cmd[5]
                pub_claw_pos.publish(values)
                pub_claw_a.publish(cmd[5])
                timer = time.time()

        elif (data[0] == 'E'):
            if data[1] == 'V':
                parameters = data[2:].strip().split(',')
                cmd = list(map(float, parameters))
                pub_cam_rot.publish(cmd[0])
                pub_cam_Pitch.publish(cmd[1])
            
            elif data[1] == 'H':
                parameters = data[2:].strip().split(',')
                cmd = float(parameters[0])
                pub_hook_a.publish(cmd)
            
            elif data[1] == 'U':
                parameters = data[2:].strip().split(',')
                cmd = float(parameters[0])
                pub_util.publish(cmd)
            
            elif data[1] == 'L':
                parameters = data[2:].strip().split(',')
                cmd = float(parameters[0])
                pub_sci.publish(cmd)

        elif (data[0] == 'S'):
            # all arm data
            parameters = data[1:].strip().split(',')
            cmd = list(map(float, parameters))

            pub_shoulder_r.publish(cmd[0])
            pub_shoulder_p.publish(cmd[1])
            pub_elbow_p.publish(cmd[2])
            pub_wrist_r.publish(cmd[3])
            pub_wrist_p.publish(cmd[4])

        elif (data[0] == 'N'):
            # Complex Control Command
            if data[1] == 'E':
                # enable complex, disable manual
                enabled = True
                pub_arm_enable.publish(True)

            elif data[1] == 'D':
                # disable complex, enable manual
                enabled = False
                pub_arm_enable.publish(False)
    

        connection.close()
    serversocket.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        serversocket.close()
