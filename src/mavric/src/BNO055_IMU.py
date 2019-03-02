#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from Adafruit_BNO055 import BNO055

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

pwm_offset_ms = 0

def angle_to_ms(angle):
    #convert from (-90 to 90) to about (1ms to 2ms)
    percent = (angle / 180.0) + 0.5
    return ((1 + percent) * 0.001) + (pwm_offset_ms * 0.001)

def servo_cal_roll(pub):
    #allow IMU to auto-calibrate roll angle
    pub.publish(angle_to_ms(-60))
    rospy.sleep(0.5)

    pub.publish(angle_to_ms(60))
    rospy.sleep(0.5)

    pub.publish(angle_to_ms(0))
    rospy.sleep(0.5)

def talker():
    global pwm_offset_ms

    #define custom 3-variable message type for this
    pub_sys_cal = rospy.Publisher("IMU/SysCalibration", Float64, queue_size=10)
    pub_cal     = rospy.Publisher("IMU/SensorCalibrations", Vector3, queue_size=10)

    pub_angle = rospy.Publisher("IMU/FusedAngle", Vector3, queue_size=10)

    pub_gyro_raw  = rospy.Publisher("IMU/RawGyro", Vector3, queue_size=10)
    pub_accel_raw = rospy.Publisher("IMU/RawAccel", Vector3, queue_size=10)
    pub_mag_raw   = rospy.Publisher("IMU/RawMag", Vector3, queue_size=10)

    pub_servo = rospy.Publisher("IMU_Cal_Servo", Float64, queue_size=10)

    rospy.init_node('BNO055_IMU')

    pwm_offset_ms = rospy.get_param("~pwm_offset_ms", 0) #0.10

    rate = rospy.Rate(20)

    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    #wiggle servo
    servo_cal_roll(pub_servo)

    while not rospy.is_shutdown():
        sys_cal, gyro_cal, accel_cal, mag_cal = bno.get_calibration_status()
        yaw, roll, pitch = bno.read_euler()

        #if necessary
        #m_x, m_y, m_z = bno.read_magnetometer()
        #g_x, g_y, g_z = bno.read_gyroscope()
        #a_x, a_y, a_z = bno.read_accelerometer()
        #                bno.read_linear_acceleration()
        #                bno.read_gravity()

        pub_sys_cal.publish(sys_cal)
        pub_cal.publish(gyro_cal, accel_cal, mag_cal)
        pub_angle.publish(roll, pitch, yaw)

        #if necessary
        #pub_gyro_raw.publish(g_x, g_y, g_z)
        #pub_accel_raw.publish(a_x, a_y, a_z)
        #pub_mag_raw.publish(m_x, m_y, m_z)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
