#!/usr/bin/env python
import rospy
from std_msgs.msg import string
from i2c import I2C

# i2c addresses for the battery checkers
batteries = [A_address, B_address]


def talker():
    pub = rospy.Publisher('Bat_Charge', String, queue_size=10)
    rospy.init_node('Voltage_Monitor', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        for i in battaries:
            voltage = read_from_ADC(i)
            bits = 1023  # number of steps
            reference = 3300  # reference voltage in mV
            voltage_divider = 8.718  # R1=68k, R2=10k
            result = ((reference * voltage_divider) / bits) * voltage  # in mV
            Rospy.loginfo(result)
            pub.publish(result)
            rate.sleep()


def read_from_ADC(address):
    bus = I2C(address, 1)
    bus.open()
    data = bus.read(10)
    bus.close()
    return data


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
