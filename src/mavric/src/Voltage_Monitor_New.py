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
import time
import board
from std_msgs.msg import Float64
from mavric.msg import Voltage
# import adafruit_ads1x15.ads1115 as ADS
# from adafruit_ads1x15 import analog_in
import Adafruit_ADS1x15 as ADS

i2c = board.I2C()

adc = ADS.ADS1115(busnum=1)
# Resistors
B1R1 = 5197
B1R2 = 995
B2R1 = 5202
B2R2 = 997
# Ratios are V_Bat/V_divider
B1Ratio = (B1R1+B1R2)/B1R2
B2Ratio = (B2R1+B2R2)/B2R2
VMax = 4.09
ADCMax = 32767



if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            adc1 = adc.read_adc(0)*VMax/ADCMax
            battery1 = adc1*B1Ratio
            print(adc1,battery1)
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass
