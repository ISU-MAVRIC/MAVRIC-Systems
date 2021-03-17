import rospy
import adafruit_bno055
from std_msgs.msg import double

from busio import I2C
from board import SDA, SCL

imui2c = busio.I2C(board.SCL, board.SDA)

imu = adafruit_bno055.BNO055_I2C(imui2c)

def imuoutput():
        imudat = rospy.Publisher('IMU_Data', double, queue_size = 10)
        rospy.init_node('IMU_Data')
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
                x, y, z = imu.euler
                imudat.publish(x,y,z)
                rate.sleep()
               

if __name__ == '__main__':
        imuoutput()
