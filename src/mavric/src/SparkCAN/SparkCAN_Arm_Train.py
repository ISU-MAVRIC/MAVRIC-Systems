#!/usr/bin/env python3
'''
Description: Reads Arm axis topics and controls the relevant motors through Mavric's SparkCAN Library
Author: Nathan Logston

TODO: Provide feedback from each arm's encoder

Topics:
    Publishers:
        NONE
    Subscribers:
        ShoulderPitch
        ShoulderRot
        ElbowPitch
        WristPitch
        WristRot
'''
import rospy
from std_msgs.msg import Float64
from mavric.msg import Armtrain
from SparkCAN import SparkBus

### scales and directions 
c_armScale = 0.25           # Governs entire arm rate
                            # if all axis are too fast or too slow, change this value

c_ShoulderPitch = 0.01      # Define individual arm rates
c_ShoulderRot = 0.5         # If one axis is faster/slower than the others, change these values
c_ElbowPitch = 0.5
c_WristPitch = 0.5
c_WristRot = 0.5

c_ShoulderRotDir = 1        # Arm Directions
c_ShoulderPitchDir = -1     # If axis is moving wrong way, invert these 
c_ElbowPitchDir = 1
c_WristPitchDir = 1
c_WristRotDir = 1

#set up globals for spark outputs. These should be zero
ShoulderRot = 0
ShoulderPitch = 0
ElbowPitch = 0
WristPitch = 0
WristRot = 0

#enable SparkBus connection to CAN0
sparkBus = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

#Setup sparks with CAN IDs
spark_shoulderPitch = sparkBus.init_controller(11)
spark_shoulderRot = sparkBus.init_controller(12)
spark_elbowPitch = sparkBus.init_controller(13)
spark_wristPitch = sparkBus.init_controller(14)
spark_wristRot = sparkBus.init_controller(15)
'''
### CALLBACK FUNCTIONS ###
Each callback function is run whenever a topic changes it's value.
When the value changes, each callback function updates its relevant axis output.
Then as a safety measure, makes sure the percent out is between the values -100 and 100.
'''
def SR_cb(data):
    global ShoulderRot
    ShoulderRot = data.data
    if ShoulderRot > 100:
        ShoulderRot = 100
    if ShoulderRot < -100:
        ShoulderRot = -100

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

def WP_cb(data):
    global WristPitch
    WristPitch = data.data
    if WristPitch > 100:
        WristPitch = 100
    if WristPitch < -100:
        WristPitch = -100

def WR_cb(data):
    global WristRot
    WristRot = data.data
    if WristRot > 100:
        WristRot = 100
    if WristRot < -100:
        WristRot = -100

def listener():
    global ShoulderRot, ShoulderPitch, ElbowPitch, WristPitch, WristRot
    rospy.init_node("CAN_ATS")

    SR_sub = rospy.Subscriber("ShoulderRot", Float64, SR_cb, queue_size=10)
    SP_sub = rospy.Subscriber("ShoulderPitch", Float64, SP_cb, queue_size=10)
    EP_sub = rospy.Subscriber("ElbowPitch", Float64, EP_cb, queue_size=10)
    WP_sub = rospy.Subscriber("WristPitch", Float64, WP_cb, queue_size=10)
    WR_sub = rospy.Subscriber("WristRot", Float64, WR_cb, queue_size=10)
    rosRate = rospy.Rate(30)
    while not rospy.is_shutdown():
        spark_shoulderRot.percent_output(c_armScale * c_ShoulderRot * ShoulderRot * c_ShoulderRotDir)
        spark_shoulderPitch.percent_output(c_armScale * c_ShoulderPitch * ShoulderPitch * c_ShoulderPitchDir)
        spark_elbowPitch.percent_output(c_armScale * c_ElbowPitch * ElbowPitch * c_ElbowPitchDir)
        spark_wristPitch.percent_output(c_armScale * c_WristPitch * WristPitch * c_WristPitchDir)
        spark_wristRot.percent_output(c_armScale * c_WristRot * WristRot * c_WristRotDir)
        rosRate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass