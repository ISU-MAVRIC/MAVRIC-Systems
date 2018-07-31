import phoenix
import rospy
from std_msgs.msg import Float64
import time

channel_vals = [-1,-1,-1,-1,-1,-1]

def callback(data, channel):
    global channel_vals
    
    channel_vals[channel] = data.data

def test():
    rover = phoenix.Phoenix('192.168.1.11')
#    rospy.init_node('DrivetrainSW')
    for i in range(6):
        rospy.Subscriber('/Drive_Board_HW/PWM_Channels/PulseWidthControl/CH'+str(i), callback, i, Float64, queue_size=10)
    
    rover.setWheels(30,30)
    time.sleep(1)
    rover.setWheels(0,0)
    time.sleep(0.1)
    for channel in channel_vals:
        assert channel==0.0015

    rover.setWheels(+50, +50)
    channel_directions = [0,0,0,0,0,0]
    time.sleep(0.1)
    for channel in range(6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            channel_directions[channel] = -1
        else:
            channel_directions[channel] = 1
    
    rover.setWheels(-50, -50)
    time.sleep(0.1)
    for channel in range(6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1
    
    
    rover.setWheels(+50, 0)
    time.sleep(0.1)
    for channel in range(3):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == -1
        else:
            assert channel_directions[channel] == 1    
    
    for channel in range(3,6):
        assert channel_vals[channel] == 0.0015


    rover.setWheels(-50, 0)
    time.sleep(0.1)
    for channel in range(3):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1    
    
    for channel in range(3,6):
        assert channel_vals[channel] == 0.0015

        
    rover.setWheels(0, +50)
    time.sleep(0.1)
    for channel in range(3):
        assert channel_vals[channel] == 0.0015
    
    for channel in range(3,6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == -1
        else:
            assert channel_directions[channel] == 1

            
    rover.setWheels(0, -50)
    time.sleep(0.1)
    for channel in range(3):
        assert channel_vals[channel] == 0.0015
    
    for channel in range(3,6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1

    return

if __name__ == '__main__':
    test()
