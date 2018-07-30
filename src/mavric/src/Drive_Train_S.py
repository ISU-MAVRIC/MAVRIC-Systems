#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from mavric.msg import Drivetrain
import time

#STOP = 50
MinTime = 0.001
MaxTime = 0.002
Range   = MaxTime-MinTime

output_topics = []

#convert from % duty cycle to PWM "ticks" for hat
def to_pulse_width(percent):
        rospy.loginfo(rospy.get_caller_id() + ": %f ms", percent);
        time = Range*percent + MinTime
        return time

# Takes in throttles as percents in the range [-1.0, +1.0]
#     LF, LM, LB, RF, RM, RB
def set_outputs(LF, LM, LB, RF, RM, RB):
        output_topics[0].publish(to_pulse_width(LF*Scale*LF_Dir/2+0.5))
        output_topics[1].publish(to_pulse_width(LM*Scale*LM_Dir/2+0.5))
        output_topics[2].publish(to_pulse_width(LB*Scale*LB_Dir/2+0.5))
        output_topics[3].publish(to_pulse_width(RF*Scale*RF_Dir/2+0.5))
        output_topics[4].publish(to_pulse_width(RM*Scale*RM_Dir/2+0.5))
        output_topics[5].publish(to_pulse_width(RB*Scale*RB_Dir/2+0.5))

def callback(data):
        print("Left: " + str(data.left) + ", Right: " + str(data.right))
        #PWM

        if (data.left > 100):
                data.left = 100
        if (data.left < -100):
                data.left = -100

        if (data.right > 100):
                data.right = 100
        if (data.right < -100):
                data.right = -100

        #get left and right side drive powers
        left  = data.left/100
        right = data.right/100

        #log values and write to PWM channels
        rospy.loginfo(rospy.get_caller_id() + " Left %s%%, Right %s%%", left*100, right*100)
        set_outputs(left, left, left, right, right, right)

def listener():
        global Scale
        global LF_Dir
        global LM_Dir
        global LB_Dir
        global RF_Dir
        global RM_Dir
        global RB_Dir

        rospy.init_node('DTS')
        rospy.Subscriber("Drive_Train", Drivetrain, callback, queue_size=10)

        output_topics.append(rospy.Publisher("LeftFront", Float64, queue_size=10))
        output_topics.append(rospy.Publisher("LeftMiddle", Float64, queue_size=10))
        output_topics.append(rospy.Publisher("LeftBack", Float64, queue_size=10))

        output_topics.append(rospy.Publisher("RightFront", Float64, queue_size=10))
        output_topics.append(rospy.Publisher("RightMiddle", Float64, queue_size=10))
        output_topics.append(rospy.Publisher("RightBack", Float64, queue_size=10))

        Scale = rospy.get_param("~Range", 0.4)
        LF_Dir = rospy.get_param("~Left_Front/Scale", 1)
        LM_Dir = rospy.get_param("~Left_Middle/Scale", 1)
        LB_Dir = rospy.get_param("~Left_Back/Scale", 1)
        RF_Dir = rospy.get_param("~Right_Front/Scale", 1) * -1
        RM_Dir = rospy.get_param("~Right_Middle/Scale", 1) * -1
        RB_Dir = rospy.get_param("~Right_Back/Scale", 1) * -1
        set_outputs(0, 0, 0, 0, 0, 0)

        rospy.spin()



if __name__ == '__main__':
    listener()
