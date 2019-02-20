#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
sensorval = 0

def sensor_callback(msg):
    global sensorval;
    sensorval = msg.data

def output(pub, val):
    print(val)
    if (val < 0.001):
        pub.publish(0.001)
    elif (val > 0.002):
        pub.publish(0.002)
        
    else:
        pub.publish(val)
    
def loop():
    rospy.init_node("control")
    pub = rospy.Publisher("/Arm_Board_HW/PWM_Channels/PulseTimeControl/CH12", Float64, queue_size=10)
    sub = rospy.Subscriber("/ADC_Channels/CH0", Float64, sensor_callback, queue_size=10)
    r = rospy.Rate(10)
    d = 0.001
    s = 1.25
    while not rospy.is_shutdown():
        e = s-sensorval
        print(e)
        output(pub, 0.0015+d*e);
        r.sleep()

loop()
