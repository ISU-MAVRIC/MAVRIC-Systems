#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64
import pigpio

pi = pigpio.pi()
bus = pi.i2c_open(1, 0x76)

adc_address = 0x76

channel_multipliers = [1, 1, 1, 1, 1, 1, 1, 1,
                       1, 1, 1, 1, 1, 1, 1, 1]

publishers = []

vref = 5
max_reading = 8388608.0

i2c_read_delay = 0.15
i2c_read_bytes = 0x06

def read_adc(adc_address, adc_channel):
    time.sleep(i2c_read_delay)
    #print(adc_channel)
    pi.i2c_write_byte(bus, (0b10110000) | (adc_channel & 0x0F))
    
    time.sleep(i2c_read_delay)

    (count, reading) = pi.i2c_read_device(bus, i2c_read_bytes)
    valor = ( ((reading[0]&0x3F) << 16) + (reading[1] << 8) + (reading[2]&0xE0) )
    volts = valor * vref / max_reading
    print(volts)

    if((reading[0] & 0b11000000) == 0b11000000):
        #handle read error
        pass

    return volts


def talker():
    for i in range(0, 2):  #supports 16, 2 for testing
        pub = rospy.Publisher("ADC_Channels/CH" + str(i), Float64, queue_size=10)
        publishers.append(pub)

    rospy.init_node('LTC2497_ADC_HAT')

    frequency = rospy.get_param("~frequency", 1/i2c_read_delay);

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        for i in range(0, 2):  #supports 16, 2 for testing
            adc_val = channel_multipliers[i] * read_adc(adc_address, i)
            publishers[i].publish(adc_val)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
