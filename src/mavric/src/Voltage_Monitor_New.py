#!/usr/bin/env python3
'''
Description:
    Reads voltages from analog to digital converter and 
    converts them to raw battery voltage for the drive
    and system batteries.
Author: 
    Nathan Logston

Topics:
    Publishers:
        Voltage_Monitor
            Batt1
            Batt2
    Subscribers:
        None
'''
import rospy
import busio
import board
from std_msgs.msg import Float64
from mavric.msg import Voltage
# import adafruit_ads1x15.ads1115 as ADS
# from adafruit_ads1x15 import analog_in
import Adafruit_ADS1x15 as ADS

i2c = board.I2C()

adc = ADS.ADS1115(busnum=1)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            battery1 = adc.read_adc(0, gain=1)
            print(battery1)
    except rospy.ROSInterruptException:
        pass
