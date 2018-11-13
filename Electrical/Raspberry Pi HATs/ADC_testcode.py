#!/usr/bin/python

######################################################################
#
# IF YOU GET IO-READ ERROR - YOU ARE MISSING THE SUCCESSIVE READ
# CAPABILITY ENABLED ON THE PI.
# For I2C and SMBus Documentation see
# http://wiki.erazor-zone.de/wiki:linux:python:smbus:doc
#
# See repeated start discussion or successive read information at
#    https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=15840&start=25
#
# sudo su -
# echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined
# exit
# sudo reboot
# File /sys/module/i2cbcm2708/parameters/combined changed from N to Y
#
# Add line
# echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined
# in /etc/rc.local before the last line. After adding the line, reboot
# the pi.
# See Pi-16ADC User Guide.
#
######################################################################
# (C) ALCHEMY POWER INC 2016,2017 etc. - ALL RIGHT RESERVED.
# CODE SUBJECT TO CHANGE AT ANY TIME.
# YOU CAN COPY/DISTRIBUTE THE CODE AS NEEDED AS LONG AS YOU MAINTAIN
# THE HEADERS (THIS PORTION) OF THE TEXT IN THE FILE.
######################################################################

import time
import smbus
import sys
import os, commands
import subprocess

from smbus import SMBus
from sys import exit

# for older PI's (version 1) use bus = SMBus(0) in statement below.
bus = SMBus(1)
#
# address is the address of the ADC chip.
# Use i2cdetect -y 1 to find the address. Use "y 0" for older Pi's.
#
# Depending on the jumper settings A0, A1 and A2, all possible combinations are shown below.
# Please refer to LTC 2495 data sheet, available at wwww.linear.com
# Page 17 of the data sheet has this information.
# Also see the Pi-16ADC User Guide for more information.
#
#
# Uncomment the line to match jumper settings for your board.
#

#  I2C Addresses for 8/16 channels (LTC2495/7/9) - note the other chips are 
# pin compatible replacements from Linear Technology.
#
#             ADDRESS     A2    A1     A0
# address =  0b0010100 #  LOW   LOW    LOW     0x14
# address =  0b0010110 #  LOW   LOW    HIGH    0x16
# address =  0b0010101 #  LOW   LOW    FLOAT   0x15
# address =  0b0100110 #  LOW   HIGH   LOW     0x26
# address =  0b0110100 #  LOW   HIGH   HIGH    0x34
# address =  0b0100111 #  LOW   HIGH   FLOAT   0x27
# address =  0b0010111 #  LOW   FLOAT  LOW     0x17
# address =  0b0100101 #  LOW   FLOAT  HIGH    0x25
# address =  0b0100100 #  LOW   FLOAT  FLOAT   0x24
# address =  0b1010110 #  HIGH  LOW    LOW     0x56
# address =  0b1100100 #  HIGH  LOW    HIGH    0x64
# address =  0b1010111 #  HIGH  LOW    FLOAT   0x57
# address =  0b1110100 #  HIGH  HIGH   LOW     0x74
#

address =  0b1110110 #  HIGH  HIGH   HIGH    0x76

#
# The above address - 0x76 is the default address for Pi-16ADC.
#
# address =  0b1110101 #  HIGH  HIGH   FLOAT   0x75
# address =  0b1100101 #  HIGH  FLOAT  LOW     0x65
# address =  0b1100111 #  HIGH  FLOAT  HIGH    0x67
# address =  0b1100110 #  HIGH  FLOAT  FLOAT   0x66
# address =  0b0110101 #  FLOAT LOW   LOW      0x35
# address =  0b0110111 #  FLOAT LOW   HIGH     0x37
# address =  0b0110110 #  FLOAT LOW   FLOAT    0x36
# address =  0b1000111 #  FLOAT HIGH  LOW      0x47
# address =  0b1010101 #  FLOAT HIGH  HIGH     0x55
# address =  0b1010100 #  FLOAT HIGH  FLOAT    0x54
# address =  0b1000100 #  FLOAT FLOAT LOW      0x44
# address =  0b1000110 #  FLOAT FLOAT HIGH     0x46
# address =  0b1000101 #  FLOAT FLOAT FLOAT    0x45
#
#

# Channel Address - Single channel use
# See LTC2497 data sheet, Table 3, Channel Selection.
# All channels are uncommented - comment out the channels you do not plan to use.
#
#
channel0        =     0xB0
channel1        =     0xB8
channel2        =     0xB1
channel3        =     0xB9
channel4        =     0xB2
channel5        =     0xBA
channel6        =     0xB3
channel7        =     0xBB
channel8        =     0xB4
channel9        =     0xBC
channel10       =     0xB5
channel11       =     0xBD
channel12       =     0xB6
channel13       =     0xBE
channel14       =     0xB7
channel15       =     0xBF
#
# Differential Channels below. Note if a differential channel is used, uncomment 2 channels above
# two channels make up a differential channel.
#
#
# The "+" and "-" signs show which channel has the positive voltage and the negative voltage.
#
#
#channel0       =       0xA0  # Differential pair Channel 0 + and Channel 1 -
#channel2       =       0xA1  # Differential pair Channel 2 + and Channel 3 -
#channel4       =       0xA2  # Differential pair Channel 4 + and Channel 5 -
#channel6       =       0xA3  # Differential pair Channel 6 + and Channel 7 -
#channel8       =       0xA4  # Differential pair Channel 8 + and Channel 9 -
#channelA       =       0xA5  # Differential pair Channel A + and Channel B -
#channelC       =       0xA6  # Differential pair Channel C + and Channel D -
#channelE       =       0xA7  # Differential pair Channel E + and Channel F -
#
#channel0       =       0xA8  # Differential pair Channel 0 - and Channel 1 +
#channel2       =       0xA9  # Differential pair Channel 2 - and Channel 3 +
#channel4       =       0xAA  # Differential pair Channel 4 - and Channel 5 +
#channel6       =       0xAB  # Differential pair Channel 6 - and Channel 7 +
#channel8       =       0xAC  # Differential pair Channel 8 - and Channel 9 +
#channelA       =       0xAD  # Differential pair Channel A - and Channel B +
#channelC       =       0xAE  # Differential pair Channel C - and Channel D +
#channelE       =       0xAF  # Differential pair Channel E - and Channel F +
#
#
#

#####################################################################################
#
# Determine the reference voltage
#
#####################################################################################
vref = 5
#####################################################################################

# To calculate the voltage, the number read in is 3 bytes. The first bit is ignored. 
# Max reading is 2^23 or 8,388,608
#

max_reading = 8388608.0

# Now we determine the operating parameters.
# lange = number of bytes to read. A minimum of 3 bytes are read in. In this sample we read in 6 bytes,
# ignoring the last 3 bytes.
# zeit (German for time) - tells how frequently you want the readings to be read from the ADC. Define the
# time to sleep between the readings.
# tiempo (Spanish - noun - for time) shows how frequently each channel is read in over the I2C bus. Best to use
# timepo between each successive readings.
#
#

lange = 0x06 # number of bytes to read in the block
zeit = 1     # number of seconds to sleep between each measurement
tiempo = 0.4 # number of seconds to sleep between each channel reading

# tiempo - has to be more than 0.2 (seconds).


#====================================================================================
# This is a subroutine which is called from the main routine. 
# The variables passed are:
# adc_address - I2C address for the device. 
#         This is set using the jumpers A0, A1 and A2. Default is 0x76
# adc_channel - the analog channel you want to read in.
#
#====================================================================================
def getreading(adc_address,adc_channel):
        bus.write_byte(adc_address, adc_channel)
        time.sleep(tiempo)
        reading  = bus.read_i2c_block_data(adc_address, adc_channel, lange)
#----------- Start conversion for the Channel Data ----------
        valor = ((((reading[0]&0x3F))<<16))+((reading[1]<<8))+(((reading[2]&0xE0)))
# Debug statements provide additional prinout information for debuggin purposes. You can leave those commented out.
# Debug
#       print("Valor is 0x%x" % valor)
#----------- End of conversion of the Channel ----------
        volts = valor*vref/max_reading

#######
# See table 1 of the LTC2947 data sheet. If the voltage > Vref/2, then it gives an error condition....
#
# So lets check the first byte
#
#######

        if( (reading[0]& 0b11000000) == 0b11000000): # we found the error
                print "*************"
                print ("Input voltage to channel 0x%x is either open or more than %5.2f. The reading may not be correct. Value read in is %12.8f Volts." % ((adc_channel), vref, volts))
                print "*************"
#       else:
                print (">>>Voltage read in on channel 0x%x is %12.8f Volts" % ((adc_channel),volts))
# Note - print is on stdout - so you can redirect that to a file if needed. You 
# can also comment out the verbiage to suite your needs.

#       time.sleep(tiempo)
# If needed pause the reading..
#
# Note - the pause is usually put in the main routine and not subroutine.
# NAK conditions are not checked for in this sample code. You can add that if needed.
#
        return volts
#====================================================================================

time.sleep(tiempo)
# Initial startup - a recommendation is to sleep tiempo seconds to give ADC time to stabilize.
# Found that it was worth it. Somehow Python and Pi get into a race situation without it. You can
# expriment omitting the initial sleep.
#
ch0_mult = 1 # This is the multiplier value to read the Current used by the Pi.
ch1_mult = 1 # Multiplier for Channel 1

# Add more multiplier for each channel as needed.

# Main routine - shows an endless loop. If used with a cron script or a 
# trigger via GPIO, an endless loop is not recommended.
#
# Here we show only channel 0 and channel 1 - you can all 16 in a loop or a 
# simple waterfall code as below.
#
while (True):
        #   Read Channel 0
        Ch0Value = ch0_mult*getreading(address, channel0)
        print ("Channel 0 at %s is %12.4f" % (time.ctime(), Ch0Value))
        # Sleep between each reading.
#        time.sleep(tiempo)
        ############# End of Channel 0 block ###################
        #   Read Channel 1
#        Ch1Value = ch1_mult*getreading(address, channel1)
#        print ("Channel 1 at %s is %12.2f" % (time.ctime(), Ch1Value))
        # Sleep between each reading.
#        time.sleep(tiempo)
        ############# End of Channel 1 block ###################
        # Write the values read should there be an interrupt. Since output is to stdout, it needs to be flushed
        # on exit or interrupt. Without that, readings could be lost.
        sys.stdout.flush()
        # Sleep till next reading.
        time.sleep(zeit)
# End of main loop.

