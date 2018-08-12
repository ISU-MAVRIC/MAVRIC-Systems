#!/usr/bin/env python
import phoenix
import rospy
from std_msgs.msg import Float64
import time

channel_vals = [0,0,0,0,0,0]
subscriptions = []
speed = 15

def callback(data, channel):
    global channel_vals    
    channel_vals[channel] = data.data

def test():
    global channel_vals
    
    rover = phoenix.Phoenix('192.168.1.11')
    rospy.init_node('DrivetrainSW')
    for i in range(3):
        topic = '/Drive_Board_HW/PWM_Channels/PulseTimeControl/CH'+str(i)
        print(topic)
        subscriptions.append(rospy.Subscriber(topic, Float64, callback, i, queue_size=speed))

    for i in range(4,7):
        topic = '/Drive_Board_HW/PWM_Channels/PulseTimeControl/CH'+str(i)
        print(topic)
        subscriptions.append(rospy.Subscriber(topic, Float64, callback, i-1, queue_size=speed))

    rover.setWheels(speed,speed)
    time.sleep(0.5)
    
    rover.setWheels(0,0)
    time.sleep(0.5)
    for channel in range(6):
        assert channel_vals[channel]==0.0015, channel_vals[channel]

    rover.setWheels(+speed, +speed)
    channel_directions = [0,0,0,0,0,0]
    time.sleep(0.5)
    for channel in range(6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            channel_directions[channel] = -1
        else:
            channel_directions[channel] = 1
    
    rover.setWheels(-speed, -speed)
    time.sleep(0.5)
    for channel in range(6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1
    
    
    rover.setWheels(+speed, 0)
    time.sleep(0.5)
    for channel in range(3):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == -1
        else:
            assert channel_directions[channel] == 1    
    
    for channel in range(3,6):
        assert channel_vals[channel] == 0.0015


    rover.setWheels(-speed, 0)
    time.sleep(0.5)
    for channel in range(3):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1    
    
    for channel in range(3,6):
        assert channel_vals[channel] == 0.0015

        
    rover.setWheels(0, +speed)
    time.sleep(0.5)
    for channel in range(3):
        assert channel_vals[channel] == 0.0015
    
    for channel in range(3,6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == -1
        else:
            assert channel_directions[channel] == 1

            
    rover.setWheels(0, -speed)
    time.sleep(0.5)
    for channel in range(3):
        assert channel_vals[channel] == 0.0015
    
    for channel in range(3,6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1

            
    rover.setWheels(speed, -speed)
    time.sleep(0.5)
    for channel in range(3):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == -1
        else:
            assert channel_directions[channel] == 1
    
    for channel in range(3,6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1

    
    rover.setWheels(-speed, speed)
    time.sleep(0.5)
    for channel in range(3):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == 1
        else:
            assert channel_directions[channel] == -1
    
    for channel in range(3,6):
        assert channel_vals[channel] != 0.0015
        if (channel_vals[channel] < 0.0015):
            assert channel_directions[channel] == -1
        else:
            assert channel_directions[channel] == 1

            
    rover.setWheels(0,0)
    
    for subscription in subscriptions:
        subscription.unregister()

    
    return

if __name__ == '__main__':
    test()
