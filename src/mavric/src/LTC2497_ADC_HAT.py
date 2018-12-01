#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64
from smbus import SMBus

bus = SMBus(1)

adc_address = 0x76

channels = [0xB0, 0xB8, 0xB1, 0xB9, 0xB2, 0xBA, 0xB3, 0xBB,
            0xB4, 0xBC, 0xB5, 0xBD, 0xB6, 0xBE, 0xB7, 0xBF]

channel_multipliers = [1, 1, 1, 1, 1, 1, 1, 1,
                       1, 1, 1, 1, 1, 1, 1, 1]

publishers = []

vref = 5
max_reading = 8388608.0

i2c_read_delay = 0.0588
i2c_read_bytes = 0x06

def read_adc(adc_address, adc_channel):
    bus.write_byte(adc_address, adc_channel)
    
    time.sleep(i2c_read_delay)

    reading = bus.read_i2c_block_data(adc_address, adc_channel, i2c_read_bytes)

    valor = ( ((reading[0]&0x3F) << 16) + (reading[1] << 8) + (reading[2]&0xE0) )
    volts = valor * vref / max_reading

    if((reading[0] & 0b11000000) == 0b11000000):
        #handle read error
        pass

    return volts


def talker():
    for i in range(0, 2):  #supports 16, 2 for testing
        pub = rospy.Publisher("ADC_Channels/CH" + str(i), Float64, queue_size=10)
        publishers.append(pub)

    rospy.init_node('LTC2497_ADC_HAT')

    frequency = rospy.get_param("~frequency", 10);

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        for i in range(0, 2):  #supports 16, 2 for testing
            adc_val = multipliers[i] * read_adc(adc_address, channels[i])
            publishers[i].publish(adc_val)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
