import rospy
import math
from std_msgs.msg import Float64


def callback(msg):

	x=msg.data

	pub-gamma.publish(math.acos((A**(2)+B**(2)-x**(2))/(2*A*B))
	pub-beta.publish(math.acos((A**(2)+x**(2)-B**(2))/(2*A*x))
	pub-alpha.publish(math.acos((B**(2)+x**(2)-A**(2))/(2*B*x))

def talker():

	global A, B

	rospy.init_node('armAngle')

	A = rospy.get_param('~A', 0)
	B = rospy.get_param('~B', 0)
	if (A <= 0):
		raise ValueError("No A value specified, or an invalid value given (A must be greater than 0).")
	if (B <= 0):
		raise ValueError("No B value specified, or an invalid value given (A must be greater than 0).")
	
	rospy.Subscriber('x', Float64, callback, queue_size=10)

	rospy.spin()

if__name__=='__main__':
	talker()
