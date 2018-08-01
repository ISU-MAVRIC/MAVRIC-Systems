#!/usr/bin/env python
# Estimates the position of the linear actuator using the limit switches and the assumed rate of travel. Currently only work with % of allowable travel, parameters can be added to make it be in arbitrary units.

# Parameters:
#   movement_speed - the assumed rate of travel of the actuator
#   forward_thresh - the minimum control value that is considered to be moving forward
#   backward thresh - the maximum control value that is considered to be moving backward

# Topics:
#   actuator_command - Subscription: the command currently being sent to the actuator,
#     this should be as far down the chain as possible. The input to the PWM driver is good.
#   switch_low - Subscription: the state of the low-side limit switch, when active, the actuator position is set to 0
#   switch_high - Subscription: the state of the high-side limit switch, when active, the actuator position is set to 1
#   actuator_state - Publication: the estimated state of the actuator.

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool

lastCommandTime = None
lastCommand = 0
movement_speed = 0
lastPosition = -1

def updatePosition():
        global lastCommandTime
        global lastCommand
        global lastPosition
        global pub
        if lastPosition == -1:
                return
        current = rospy.Time.now()
        if lastCommandTime == None:
                delta = rospy.Time(1)
        else:
                delta = current - lastCommandTime
        if lastCommand == -1:
                lastPosition = lastPosition - movement_speed*delta.to_sec()
        elif lastCommand == 1:
                lastPosition = lastPosition + movement_speed*delta.to_sec()
        
        pub.publish(lastPosition)
        lastCommandTime = current

def callback(data):
        global lastCommand
        updatePosition()
        if data.data > forward_thresh:
                lastCommand = 1
        elif data.data < backward_thresh:
                lastCommand = -1
        else:
                lastCommand = 0

def switch_callback(data, switch):
        global lastPosition
        
        updatePosition()
        if data.data:
                if switch == 0:
                        lastPosition = 0
                elif switch == 1:
                        lastPosition = 1

def talker():
        global pub
        global movement_speed
        global forward_thresh
        global backward_thresh
        
	rospy.init_node('Actuator Status')
        rospy.Subscriber("actuator_command", Float64, callback, queue_size=10)
        rospy.Subscriber("switch_low", Bool, switch_callback, 0, queue_size=10)
        rospy.Subscriber("switch_high", Bool, switch_callback, 1, queue_size=10)
        movement_speed = rospy.get_param('~movement_speed', 0.05)
        forward_thresh = rospy.get_param('~forward_thresh', 0.0016)
        backward_thresh = rospy.get_param('~backward_thresh', 0.0014)
        pub = rospy.Publisher('actuator_status', Float64, queue_size=10, latch=True)
        pub.publish(-1)
        rospy.spin()
                
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

