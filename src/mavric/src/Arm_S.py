#!/usr/bin/env python
# Arm control, listens to messages on the Arm topic and splits them into commands for each joint
# Eventually this will take in an absolute motion and perform inverse kinematics to achieve that movement

import rospy
from std_.msgs.msg import Float64
from mavric.msg import Arm

output_topics = []

def clear_outputs():
    output_topics[0].publish(0)
    output_topics[1].publish(0)
    output_topics[2].publish(0)
    output_topics[3].publish(0)
    output_topics[4].publish(0)
    output_topics[5].publish(0)

def cb_shoulder_r(data):
    output_topics[0].publish(data.shoulder_rotation_rate * scale * base_r_dir)

def cb_shoulder_p(data):
    output_topics[1].publish(data.shoulder_pitch_rate * scale * base_p_dir)

def cb_elbow_p(data):
    output_topics[2].publish(data.elbow_pitch_rate * scale * elbow_p_dir)

def cb_wrist_r(data):
    output_topics[3].publish(data.wrist_rotation_rate * scale * wrist_r_dir)

def cb_wrist_p(data):
    output_topics[4].publish(data.wrist_pitch_rate * scale * wrist_p_dir)

def cb_claw_a(data):
    output_topics[5].publish(data.claw_actuation_rate * scale * claw_a_dir)
    

def listener():
    global scale
    global base_r_dir
    global base_p_dir
    global elbow_p_dir
    global wrist_r_dir
    global wrist_p_dir
    global claw_a_dir

    rospy.init_node('ARS')
    rospy.Subscriber("Arm_ShoulderRot", Arm, cb_shoulder_r, queue_size=10)
    rospy.Subscriber("Arm_ShoulderPitch", Arm, cb_shoulder_p, queue_size=10)
    
    rospy.Subscriber("Arm_ElbowPitch", Arm, cb_elbow_p, queue_size=10)
    
    rospy.Subscriber("Arm_WristRot", Arm, cb_wrist_r, queue_size=10)
    rospy.Subscriber("Arm_WristPitch", Arm, cb_wrist_p, queue_size=10)
    
    rospy.Subscriber("Arm_ClawActuation", Arm, cb_claw_a, queue_size=10)


    output_topics.append(rospy.Publisher("ShoulderRot", Float64, queue_size=10))
    output_topics.append(rospy.Publisher("ShoulderPitch", Float64, queue_size=10))
    
    output_topics.append(rospy.Publisher("ElbowPitch", Float64, queue_size=10))

    output_topics.append(rospy.Publisher("WristRot", Float64, queue_size=10))
    output_topics.append(rospy.Publisher("WristPitch", Float64, queue_size=10))

    output_topics.append(rospy.Publisher("ClawActuation", Float64, queue_size=10))


    scale = rospy.get_param("~Range", 0.4)

    base_r_dir = rospy.get_param("~ShoulderRot/Scale", 1);
    base_p_dir = rospy.get_param("~ShoulderPitch/Scale", 1);
    elbow_p_dir = rospy.get_param("~ElbowPitch/Scale", 1);
    wrist_r_dir = rospy.get_param("~WristRot/Scale", 1);
    wrist_p_dir = rospy.get_param("~WristPitch/Scale", 1);
    claw_a_dir = rospy.get_param("~ClawActuation/Scale", 1);

    clear_outputs()
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    listener()
