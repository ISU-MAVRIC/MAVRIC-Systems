#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist
import math as m

delta_linear = [0.0, 0.0, 0.0]
delta_angular = [0.0, 0.0, 0.0]
pos_linear = [0.0, 0.0, 0.0]
pos_angular = [0.0, 0.0, 0.0]

def callback(data):
    global delta_linear, delta_angular
    delta_linear = [data.linear.x, data.linear.y, data.linear.z]
    delta_angular = [data.angular.x, data.angular.y, data.angular.z]

def talker():
    global delta_linear, delta_angular
    rospy.init_node('Arm_Calculator')
    c_l = rospy.get_param('~claw_length', 0.163)
    l_l = rospy.get_param('~lower_length', 0.541)
    u_l = rospy.get_param('~upper_length', 0.576)
    pos_linear = rospy.get_param('~start_linear', [0.739, 0, 0.541])
    pos_angular = rospy.get_param('~start_angular', [0, 0])
    sr_r2p = rospy.get_param('~ShoulderRotRad2Pulse', 350.1409)
    sp_r2p = rospy.get_param('~ShoulderPitchRad2Pulse', 350.1409)
    ep_r2p = rospy.get_param('~ElbowPitchRad2Pulse', 350.1409)
    wr_r2p = rospy.get_param('~WristRotRad2Pulse', 350.1409)
    wp_r2p = rospy.get_param('~WristPitchRad2Pulse', 912.9128)

    rospy.Subscriber('ClawPosition', Twist, callback, queue_size=10)
    pub_sr = rospy.Publisher('ShoulderRotPos', Float64, queue_size=10)
    pub_sp = rospy.Publisher('ShoulderPitchPos', Float64, queue_size=10)
    pub_ep = rospy.Publisher('ElbowPitchPos', Float64, queue_size=10)
    pub_wr = rospy.Publisher('WristRotPos', Float64, queue_size=10)
    pub_wp = rospy.Publisher('WristPitchPos', Float64, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        global delta_linear, delta_angular
        alpha = pos_angular[0] + delta_angular[0]
        beta = pos_angular[1] + delta_angular[1]
        x = pos_linear[0] + delta_linear[0]
        y = pos_linear[1] + delta_linear[1]
        z = pos_linear[2] + delta_linear[2]
        mu = m.atan(y/x)
        u =  x - c_l*m.cos(mu)*m.cos(alpha)
        w =  z - c_l*m.cos(mu)*m.sin(alpha)
        #print(u)
        #print(w)
        if 0.01 < m.sqrt(m.pow(u, 2)+m.pow(w, 2)) < 0.98*(u_l+l_l):
            B = m.sqrt(m.pow(u, 2) + m.pow(w, 2))
            b = m.acos((m.pow(u_l, 2) + m.pow(l_l, 2) - m.pow(B, 2)) / (2 * u_l * l_l))
            a = m.asin(u_l * m.sin(b) / B)
            c = m.asin(l_l * m.sin(b) / B)
            theta = m.pi/2 - (a + m.atan(w / u))
            phi = m.pi/2 - b
            gamma = alpha + c + m.atan(u / w) - m.pi/2
            pub_sr.publish(mu*sr_r2p)
            pub_sp.publish(theta*sp_r2p)
            pub_ep.publish(phi*ep_r2p)
            pub_wp.publish(gamma*wp_r2p)
            pub_wr.publish(beta*wr_r2p)
            pos_linear[0] = x
            pos_linear[1] = y
            pos_linear[2] = z
            pos_angular[0] = alpha
            pos_angular[1] = beta
            delta_linear = [0, 0, 0]
            delta_angular = [0, 0, 0]
        else:
            delta_linear = [0, 0, 0]
            delta_angular = [0, 0, 0]
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
