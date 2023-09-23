#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from mavric.msg import Armtrain
from SparkCAN import SparkBus

#scales and directions
c_armScale = 0.25
c_ShoulderPitch = 0.05
c_ElbowPitch = 0.5
c_ShoulderRotDir = 1
c_ShoulderPitchDir = -1
c_ElbowPitchDir = -1
c_WristPitchDir = 1
c_WristRotDir = 1

#set up globals for spark outputs
ShoulderRot = 0
ShoulderPitch = 0
ElbowPitch = 0
WristPitch = 0
WristRot = 0

pRateUp = 100
pRateDown = 100

#enable SparkBus connection to CAN0
sparkBus = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

#Setup sparks with CAN IDs
spark_shoulderPitch = sparkBus.init_controller(11)
spark_shoulderRot = sparkBus.init_controller(12)
spark_elbowPitch = sparkBus.init_controller(13)
spark_wristPitch = sparkBus.init_controller(14)
spark_wristRot = sparkBus.init_controller(15)

def SP_cb(data):
    global ShoulderPitch
    ShoulderPitch = data.data
    if ShoulderPitch > 100:
        ShoulderPitch = 100
    if ShoulderPitch < -100:
        ShoulderPitch = -100

def EP_cb(data):
    global ElbowPitch
    ElbowPitch = data.data
    if ElbowPitch > 100:
        ElbowPitch = 100
    if ElbowPitch < -100:
        ElbowPitch = -100

def listener():
    global ShoulderPitch
    rospy.init_node("CAN_ATS")

    #SR_sub = rospy.Subscriber("ShoulderRot", Float64, SR_cb, queue_size=10)
    SP_sub = rospy.Subscriber("ShoulderPitch", Float64, SP_cb, queue_size=10)
    EP_sub = rospy.Subscriber("ElbowPitch", Float64, EP_cb, queue_size=10)
    rosRate = rospy.Rate(30)
    while not rospy.is_shutdown():
        spark_shoulderPitch.percent_output(c_armScale * c_ShoulderPitch * ShoulderPitch * c_ShoulderPitchDir)
        spark_elbowPitch.percent_output(c_armScale * c_ElbowPitch * ElbowPitch * c_ElbowPitchDir)
        rosRate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass