#include "ros/ros.h"
#include "std_msgs/Float64.h"

void tstCallback(const std_msgs::Float64::ConstPtr &msg)
{
	ROS_INFO("Heard: %lf", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CAN_DTS");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/CAN/lol", 1000, tstCallback);
	ros::spin();

	return 0;
}
