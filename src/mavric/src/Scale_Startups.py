#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64
from mavric.msg import ArmData

def talker():
    rospy.init_node("Scale_Defaults")
    drive_sens = rospy.Publisher("Drive/Drive_Sensitivity", Float64, queue_size=10)
    arm_sens = rospy.Publisher("Arm/Arm_Sensitivity", ArmData, queue_size=10)
    arm_values = ArmData()
    drive_value = Float64()
    arm_values.ShoulderRot = rospy.get_param('~ShoulderRot', '1.0')
    arm_values.ShoulderPitch = rospy.get_param('~ShoulderPitch', '1.0')
    arm_values.ElbowPitch = rospy.get_param('~ElbowPitch', '1.0')
    arm_values.WristPitch = rospy.get_param('~WristPitch', '1.0')
    arm_values.WristRot = rospy.get_param('~WristRot', '1.0')
    drive_value = rospy.get_param('~Drive_Sens','1.0')
    time.sleep(5)
    arm_sens.publish(arm_values)
    #drive_sens.publish(drive_value)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
