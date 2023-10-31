#!/usr/bin/env python
import rospy
import time
import os
from pySerialTransfer import pySerialTransfer as txfer
import driveMath


from mavric.msg import Drivetrain
from mavric.msg import Steertrain

port = '/dev/ttyACM0'

drive = 0.0
steer = 0.0


class struct(object):
    drive = float
    steer = float

if __name__=='__main__':
    try:
        drive = rospy.Publisher("Drive_Train", Drivetrain, queue_size=10)
        steer = rospy.Publisher("Steer_Train", Steertrain, queue_size=10)
        rospy.init_node("LoRa_Radio")
        dataRX = struct
        print("Attempting to Connect to {}".format(port))
        link = txfer.SerialTransfer(port)
        link.open()
        print("Connected to {}".format(port))
        time.sleep(1)

        parameters = [0,0,0,0,0,0,0,0,0,0]

        while True:
            if link.available():
                recSize = 0

                dataRX.drive = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                dataRX.steer = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                #print('drive: {}  steer: {}'.format(dataRX.drive, dataRX.steer)) # debug
            elif link.status < 0:
                if link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status))
            dataRX.drive = round(dataRX.drive, 2)
            dataRX.steer = round(dataRX.steer, 2)
            #print("Drive: {}    Steer: {}".format(dataRX.drive,dataRX.steer))
            parameters = driveMath.car_drive(dataRX.drive,dataRX.steer)
            for i in range(8):
                if parameters[i] < 0.05:
                    parameters[i] = 0
            drive.publish(float(parameters[0]), float(parameters[1]), float(parameters[2]), float(parameters[3]), float(parameters[4]), float(parameters[5]))
            steer.publish(float(parameters[6]), float(parameters[7]), float(parameters[8]), float(parameters[9]))
    except rospy.ROSInterruptException:
        pass
