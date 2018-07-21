#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mavric.msg import Drivetrain
import Adafruit_PCA9685
import time

#STOP = 50
MinTime = 0.001
MaxTime = 0.002
Range   = MaxTime-MinTime
Period  = 0.003609
STOP    = MinTime + Range/2

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(1/0.004)

#convert from % duty cycle to PWM "ticks" for hat
def to_tick(percent):
        rospy.loginfo(rospy.get_caller_id() + ": %f%%", percent*100);
        time = Range*percent + MinTime
        percent = time/Period
        # the 0.92 is a fudge factor that is needed for unokown reasons
        return int(4095 * percent)

# Takes in throttles as percents in the range [-1.0, +1.0]
#     LF, LM, LB, RF, RM, RB
def set_outputs(LF, LM, LB, RF, RM, RB):
        global LF_Chan
        global LM_Chan
        global LB_Chan
        global RF_Chan
        global RM_Chan
        global RB_Chan
        global Scale
        global LF_Dir
        global LM_Dir
        global LB_Dir
        global RF_Dir
        global RM_Dir
        global RB_Dir


        if (LF_Chan >= 0):
                pwm.set_pwm(LF_Chan, 0, to_tick(LF*Scale*LF_Dir/2+0.5))
        if (LM_Chan >= 0):
                pwm.set_pwm(LM_Chan, 0, to_tick(LM*Scale*LM_Dir/2+0.5))
        if (LB_Chan >= 0):
                pwm.set_pwm(LB_Chan, 0, to_tick(LB*Scale*LB_Dir/2+0.5))
        if (RF_Chan >= 0):
                pwm.set_pwm(RF_Chan, 0, to_tick(RF*Scale*RF_Dir/2+0.5))
        if (RM_Chan >= 0):
                pwm.set_pwm(RM_Chan, 0, to_tick(RM*Scale*RM_Dir/2+0.5))
        if (RB_Chan >= 0):
                pwm.set_pwm(RB_Chan, 0, to_tick(RB*Scale*RB_Dir/2+0.5))
        return

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
        global LF_Chan
        global LM_Chan
        global LB_Chan
        global RF_Chan
        global RM_Chan
        global RB_Chan
        global Scale
        global LF_Dir
        global LM_Dir
        global LB_Dir
        global RF_Dir
        global RM_Dir
        global RB_Dir

        rospy.init_node('DTS')
        rospy.Subscriber("/Drive_Train", Drivetrain, callback, queue_size=10)
        #time.sleep(10)
        LF_Chan = rospy.get_param("/Drive/Left_Front/Channel", -1)
        LM_Chan = rospy.get_param("/Drive/Left_Middle/Channel", -1)
        LB_Chan = rospy.get_param("/Drive/Left_Back/Channel", -1)
        RF_Chan = rospy.get_param("/Drive/Right_Front/Channel", -1)
        RM_Chan = rospy.get_param("/Drive/Right_Middle/Channel", -1)
        RB_Chan = rospy.get_param("/Drive/Right_Back/Channel", -1)
        Scale = rospy.get_param("/Drive/Range", 0.4)
        LF_Dir = rospy.get_param("/Drive/Left_Front/Dir", 1)
        LM_Dir = rospy.get_param("/Drive/Left_Middle/Dir", 1)
        LB_Dir = rospy.get_param("/Drive/Left_Back/Dir", 1)
        RF_Dir = rospy.get_param("/Drive/Right_Front/Dir", 1) * -1
        RM_Dir = rospy.get_param("/Drive/Right_Middle/Dir", 1) * -1
        RB_Dir = rospy.get_param("/Drive/Right_Back/Dir", 1) * -1
        set_outputs(0, 0, 0, 0, 0, 0)

        rospy.spin()



if __name__ == '__main__':
listener()
