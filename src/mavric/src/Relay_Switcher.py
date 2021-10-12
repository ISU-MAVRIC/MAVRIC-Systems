import rospy
from std_msgs.msg import Bool

config = {
    "Lightbar": 1
}

def callback(data):
    print(data)

def talker():
        rospy.init_node('Float64_Heartbeat')
        for relay_name, relay_pin in config.items():
            rospy.Subscriber('/Relay/' + relay_name, Bool, callback, (relay_name, relay_pin))
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
                r.sleep()

if __name__ == '__main__':
    talker()
