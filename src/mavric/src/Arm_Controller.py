#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math as m

delta_linear = [0, 0, 0]
delta_angular = [0, 0, 0]
pos_linear = [0, 0, 0]
pos_angular = [0, 0, 0]

def callback(data):
    global delta_linear, delta_angular
    delta_linear = data.linear
    delta_angular = data.delta_angular

def talker():
    global delta_linear, delta_angular
    rospy.init_node('Arm_Calculator')
    c_l = rospy.get_param('~claw_length', 0.25)
    l_l = rospy.get_param('~lower_length', 1)
    u_l = rospy.get_param('~upper_length', 1)
    pos_linear = rospy.get_param('~start_linear', [1, 0, 1])
    pos_angular = rospy.get_param('~start_angular', [0, 0])

    rospy.Subscriber('ClawPosition', Twist, callback, queue_size=10)
    pub_sr = rospy.Publisher('ShoulderRotPos', Float64, queue_size=10)
    pub_sp = rospy.Publisher('ShoulderPitchPos', Float64, queue_size=10)
    pub_ep = rospy.Publisher('ElbowPitchPos', Float64, queue_size=10)
    pub_wr = rospy.Publisher('WristRotPos', Float64, queue_size=10)
    pub_wr = rospy.Publisher('ShoulderRotPos', Float64, queue_size=10)

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
        if 0.02 < m.sqrt(m.pow(u, 2)+m.pow(w, 2)) < 0.98*m.sqrt(m.pow(u_l, 2)+m.pow(l_l, 2)):
            B = m.sqrt(m.pow(u, 2) + m.pow(w, 2))
            b = m.acos((m.pow(u_l, 2) + m.pow(l_l, 2) - m.pow(B, 2)) / (2 * u_l * l_l))
            a = m.asin(u_l * m.sin(b) / B)
            c = m.asin(l_l * m.sin(b) / B)
            theta = a + m.atan(w / u)
            phi = m.pi/2 - b
            gamma = alpha + c + m.atan(u / w) - m.pi/2
            pub_sr.publish(mu)
            pub_sp.publish(theta)
            pub_ep.publish(phi)
            pub_wp.publish(gamma)
            pub_wr.publish(beta)
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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass