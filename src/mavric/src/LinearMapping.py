#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool

def callback(data, (pub, slope, intercept)):
        pub.publish(data.data*slope+intercept)

def talker():
        global low_limit
        global slope
        global intercept
        
        rospy.init_node('LinearMapping')
        inputs = rospy.get_param('~inputs', '').split(',')
        outputs = rospy.get_param('~outputs', '').split(',')

        slopes = rospy.get_param('~slopes', '0')
        if type(slopes) == str:
                slopes = map(float,slopes.split(','))
        else:
                slopes = [slopes]
                
        intercepts = rospy.get_param('~intercepts', '0')
        if type(intercepts) == str:
                intercepts = map(float,intercepts.split(','))
        else:
                intercepts = [intercepts]

        if len(inputs) != len(outputs):
                raise ValueError("The inputs and outputs are not the same size")
        if len(inputs) != len(slopes):
                raise ValueError("The inputs and slopes are not the same size")
        if len(inputs) != len(intercepts):
                raise ValueError("The inputs and intercepts are not the same size")
                                 
        for i in range(len(inputs)):
                output_topic = rospy.Subscriber(inputs[i], Float64, callback, (rospy.Publisher(outputs[i], Float64, queue_size=10), slopes[i], intercepts[i]), queue_size=10)

        rospy.spin()
        
if __name__ == '__main__':
    talker()
