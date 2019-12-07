#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64
from i2c import I2C

vref = 5
max_reading = 2**12-1.0
i2c_read_bytes = 6

def read_adc(adc, adc_channel):
    #print(adc_channel)
    sign = adc_channel%2
    sel = adc_channel/2
    message = bytearray()
    message.append(0b10001000 | (sign << 6) | (sel << 4))
    #print(len(message))
    adc.write(message)

    reading = adc.read(i2c_read_bytes)
    valor = (((reading[0]) << 4) + (reading[1] >> 4))
    volts = valor * vref / max_reading

    return volts


def talker():
    rospy.init_node('LTC2497_ADC_HAT')
    publishers = []
    for i in range(0, 8):  #supports 8
        pub = rospy.Publisher("ADC_Channels/CH" + str(i), Float64, queue_size=10)
        publishers.append(pub)

    frequency = rospy.get_param("~frequency", 100);
    i2c_address = rospy.get_param("~address", 0x18);
    rate = rospy.Rate(frequency)
    adc = I2C(i2c_address, 1)

    while not rospy.is_shutdown():
        for i in range(0, 8):
            adc_val = read_adc(adc, i)
            publishers[i].publish(adc_val)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
