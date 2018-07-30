#!/usr/bin/env python
# Drivetrain control, listens to messages on the Drive_Train topic and splits them into commands for each wheel.
# May eventually add features like smarter turning.
# Ramping:
#   The ramping is set to allow a wheel to return to 0 immediately, but then ramp from there.
#   This means that if a wheel is set to forwards, then moved backwards, it will try to stop immediately, but take some time to accelerate backwards.

import rospy
from std_msgs.msg import Float64
from mavric.msg import Drivetrain
import time

#STOP = 50
MinTime = 0.001
MaxTime = 0.002
Range   = MaxTime-MinTime

output_topics = []
left_target = 0
right_target = 0

#convert from % duty cycle to PWM "ticks" for hat
def to_pulse_width(percent):
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
        global left_target
        global right_target

        if (data.left > 100):
                data.left = 100
        if (data.left < -100):
                data.left = -100

        if (data.right > 100):
                data.right = 100
        if (data.right < -100):
                data.right = -100

        #get left and right side drive powers
        left_target  = data.left/100.
        right_target = data.right/100.

        #log values and write to PWM channels
#        rospy.loginfo(rospy.get_caller_id() + " Left %s%%, Right %s%%", left*100, right*100)
#        set_outputs(left, left, left, right, right, right)

def rampVal(current, target, ramp_amount):
        if current >= 0 and target > 0:
                if current < target:
                        current = current + min(ramp_amount, target-current)
                else:
                        current = target
        elif current > 0 and target < 0:
                current = 0
        elif current <= 0 and target < 0:
                if current > target:
                        current = current - min(ramp_amount, current-target)
                else:
                        current = target
        elif current < 0 and target > 0:
                current = 0
        else:
                current = target;
        return current
        
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
        ramp_rate = rospy.get_param("~ramp_rate", 0.5)/20
        set_outputs(0, 0, 0, 0, 0, 0)
        r = rospy.Rate(20)
        left = 0
        right = 0
        
        while not rospy.is_shutdown():
                if (left != left_target or right != right_target):
                        left = rampVal(left, left_target, ramp_rate)
                        right = rampVal(right, right_target, ramp_rate)
                        set_outputs(left, left, left, right, right, right)
                        
                r.sleep()
                
if __name__ == '__main__':
    listener()
