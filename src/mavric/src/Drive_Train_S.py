#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import Adafruit_PCA9685

#STOP = 50
MinTime = 0.001
MaxTime = 0.002
Range   = MaxTime-MinTime
Period  = 0.004
STOP    = MinTime + Range/2

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(1/Period)

#convert from % duty cycle to PWM "ticks" for hat
def to_tick(percent):
        rospy.loginfo(rospy.get_caller_id() + ": %f%%", percent*100);
        time = Range*percent + MinTime
        percent = time/Period
        # the 0.92 is a fudge factor that is needed for unokown reasons
	return int(4095 * percent / 0.92)

# Takes in throttles as percents in the range [-1.0, +1.0]
#     LF, LM, LB, RF, RM, RB
def set_outputs(LF, LM, LB, RF, RM, RB):
        LF_Chan = rospy.get_param("/Drive/Left_Front_Channel", -1)
        LM_Chan = rospy.get_param("/Drive/Left_Middle_Channel", -1)
        LB_Chan = rospy.get_param("/Drive/Left_Back_Channel", -1)
        RF_Chan = rospy.get_param("/Drive/Right_Front_Channel", -1)
        RM_Chan = rospy.get_param("/Drive/Right_Middle_Channel", -1)
        RB_Chan = rospy.get_param("/Drive/Right_Back_Channel", -1)
        Scale = rospy.get_param("/Drive/Range", -0.2)
        
        if (LF_Chan >= 0):
                pwm.set_pwm(LF_Chan, 0, to_tick(LF*Scale/2+0.5))
        if (LM_Chan >= 0):
                pwm.set_pwm(LM_Chan, 0, to_tick(LM*Scale/2+0.5))
        if (LB_Chan >= 0):
                pwm.set_pwm(LB_Chan, 0, to_tick(LB*Scale/2+0.5))
        if (RF_Chan >= 0):
                pwm.set_pwm(RF_Chan, 0, to_tick(RF*Scale/2+0.5))
        if (RM_Chan >= 0):
                pwm.set_pwm(RM_Chan, 0, to_tick(RM*Scale/2+0.5))
        if (RB_Chan >= 0):
                pwm.set_pwm(RB_Chan, 0, to_tick(RB*Scale/2+0.5))
        return

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)
	#PWM
	
	#if the data string is invalid, kill motors
        data.data = data.data.strip()
	if(len(data.data) != 5):
		rospy.loginfo(rospy.get_caller_id() + " STOPPING")
                set_outputs(0, 0, 0, 0, 0, 0)
		return
	
	#get left and right side drive powers
	left  = (int(data.data[1:3])-50)/50.0
	right = (int(data.data[3:5])-50)/50.0
	
	#log values and write to PWM channels
	rospy.loginfo(rospy.get_caller_id() + " Left %s%%, Right %s%%", left*100, right*100)
        set_outputs(left, left, left, right, right, right)

        

def listener():
	rospy.init_node('DTS', anonymous=True)
	rospy.Subscriber("/Drive_Train", String, callback)
        
        set_outputs(0, 0, 0, 0, 0, 0)
        
	rospy.spin()


if __name__ == '__main__':
	listener()







