# Calibration script to get and set IMU registers
# 10/9/2023 Started by Colin Greiner
# Revised 10/9/2023
# Iowa State MAVRIC - M2I
## Code is inspired by
## https://github.com/adafruit/Adafruit_Python_BNO055/blob/ce9d7ae76cb23b8962d2dddc48f943bd8416831d/Adafruit_BNO055/BNO055.py#L544

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import adafruit_bno055 as BNO055
import board
import time
import json
import random

i2c = board.I2C()
bno = BNO055.BNO055_I2C(i2c)

pwm_offset_ms = 0
HEADING_OFFSET = 0

accel_offsets = [0X55, 0X56, 0X57, 0X58, 0X59, 0X5A]
mag_offsets = [0X5B, 0X5C, 0X5D, 0X5E, 0X5F, 0X60]
gyro_offsets = [0X61, 0X62, 0X63, 0X64, 0X65, 0X66]
radius_offsets = [0X67, 0X68, 0X69, 0X6A]

CONFIG_MODE = 0x00
NDOF_MODE = 0x0C
AMG_MODE = 0x07
NDOF_FMC_OFF_MODE = 0x0B

MODES_LIST = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]

register_offsets_master = accel_offsets + mag_offsets + gyro_offsets + radius_offsets

def get_calibration(last_mode) -> list:
    print("last: "+str(last_mode))
    bno.mode = CONFIG_MODE
    print("config: "+str(bno.mode))
    data = list()
    for i in register_offsets_master:
        data.append(bno._read_register(i))
    bno.mode = last_mode
    print("back to last: "+str(bno.mode))
    return data

def set_calibration(data, last_mode):
    bno.mode = CONFIG_MODE
    print("config: "+str(bno.mode))
    for i in range(0,len(register_offsets_master)):
        bno._write_register(register_offsets_master[i], data[i])
    bno.mode = last_mode
    print("NDOF: "+str(bno.mode))
    print("setting_calibration")

def print_data(data):
    print()
    for i in range(0,len(data)):
        print("d:"+str(data[i])+"  ",end="")
    print()

def generate_random_offsets() -> list:
    data = list()
    for i in range(0,len(register_offsets_master)):
        data.append(random.randint(0,255))
    return data

def compare_data(list_1, list_2) -> list:
    similiar = list()
    for i in range(0,len(list_1)):
        if list_1[i] == list_2[i]:
            similiar.append(list_1[i])
    return similiar

def main():
    print("starting . . .")
    while True:
        random_number = generate_random_offsets()
        for i in range(0,len(MODES_LIST)):
            print("\n\n --- next read ---  ")

            set_calibration(random_number, MODES_LIST[i])
            print_data(random_number)

            data = get_calibration(MODES_LIST[i])
            print_data(data)

            similiar_data = compare_data(random_number, data)
            similiar_result = len(similiar_data)
            print("Number of Similiar " + str(similiar_result))
        time.sleep(7.5)

if __name__ == '__main__':
    main()

    
#def print_IMU_info(cal):
#        sys_cal, gyro_cal, accel_cal, mag_cal = bno.calibration_status
#        yaw, pitch, roll = bno.euler
#        if cal == True and yaw != None:
#            HEADING_OFFSET = -yaw
#            cal = False
#        print("sys_cal: {}  gyro_cal: {}  accel_cal: {}  mag_cal: {} ".format(sys_cal, gyro_cal, accel_cal, mag_cal))
#        print("yaw: {}  pitch: {}  roll: {}".format(yaw, pitch, roll))
    
        
#unimplimented
#def save_calibration():
#    # Save calibration data to disk.
#    # First grab the lock on BNO sensor access to make sure nothing else is
#    # writing to the sensor right now.
#    with bno_changed:
#        data = bno.get_calibration()
#    # Write the calibration to disk.
#    with open(CALIBRATION_FILE, 'w') as cal_file:
#        json.dump(data, cal_file)
#    return 'OK'

        

## Accelerometer Offset registers
#ACCEL_OFFSET_X_LSB_ADDR              = 0X55
#ACCEL_OFFSET_X_MSB_ADDR              = 0X56
#ACCEL_OFFSET_Y_LSB_ADDR              = 0X57
#ACCEL_OFFSET_Y_MSB_ADDR              = 0X58
#ACCEL_OFFSET_Z_LSB_ADDR              = 0X59
#ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A
#
## Magnetometer Offset registers
#MAG_OFFSET_X_LSB_ADDR                = 0X5B
#MAG_OFFSET_X_MSB_ADDR                = 0X5C
#MAG_OFFSET_Y_LSB_ADDR                = 0X5D
#MAG_OFFSET_Y_MSB_ADDR                = 0X5E
#MAG_OFFSET_Z_LSB_ADDR                = 0X5F
#MAG_OFFSET_Z_MSB_ADDR                = 0X60
#
## Gyroscope Offset register s
#GYRO_OFFSET_X_LSB_ADDR               = 0X61
#GYRO_OFFSET_X_MSB_ADDR               = 0X62
#GYRO_OFFSET_Y_LSB_ADDR               = 0X63
#GYRO_OFFSET_Y_MSB_ADDR               = 0X64
#GYRO_OFFSET_Z_LSB_ADDR               = 0X65
#GYRO_OFFSET_Z_MSB_ADDR               = 0X66
#
## Radius registers
#ACCEL_RADIUS_LSB_ADDR                = 0X67
#ACCEL_RADIUS_MSB_ADDR                = 0X68
#MAG_RADIUS_LSB_ADDR                  = 0X69
#MAG_RADIUS_MSB_ADDR                  = 0X6A
#
#
#
#        
#    #cal_data = list(bno._read_register()
##
##
#    #def get_calibration(self):
#    #    """Return the sensor's calibration data and return it as an array of
    #    22 bytes. Can be saved and then reloaded with the set_calibration function
    #    to quickly calibrate from a previously calculated set of calibration data.
    #    """
    #    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    #    self._config_mode()
    #    # Read the 22 bytes of calibration data and convert it to a list (from
    #    # a bytearray) so it's more easily serialized should the caller want to
    #    # store it.
    #    cal_data = list(self._read_bytes(ACCEL_OFFSET_X_LSB_ADDR, 22))
    #    # Go back to normal operation mode.
    #    self._operation_mode()
    #    return cal_data
#
    #def set_calibration(self, data):
    #    """Set the sensor's calibration data using a list of 22 bytes that
    #    represent the sensor offsets and calibration data.  This data should be
    #    a value that was previously retrieved with get_calibration (and then
    #    perhaps persisted to disk or other location until needed again).
    #    """
    #    # Check that 22 bytes were passed in with calibration data.
    #    if data is None or len(data) != 22:
    #        raise ValueError('Expected a list of 22 bytes for calibration data.')
    #    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    #    self._config_mode()
    #    # Set the 22 bytes of calibration data.
    #    self._write_bytes(ACCEL_OFFSET_X_LSB_ADDR, data)
    #    # Go back to normal operation mode.
    #    self._operation_mode()
    #    
    #    
    #def calibration_status(self) -> Tuple[int, int, int, int]:
    #    """Tuple containing sys, gyro, accel, and mag calibration data."""
    #    calibration_data = self._read_register(_CALIBRATION_REGISTER)
    #    sys = (calibration_data >> 6) & 0x03
    #    gyro = (calibration_data >> 4) & 0x03
    #    accel = (calibration_data >> 2) & 0x03
    #    mag = calibration_data & 0x03
    #    return sys, gyro, accel, mag