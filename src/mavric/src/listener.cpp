#include "ros/ros.h"
#include "std_msgs/String.h"

void mavric_debug_callback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Message Recieved: %s", msg->data.c_str());
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mstr_cntl_brd_debug");
  ros::NodeHandle thisNode;

  ros::Subscriber subscription = thisNode.subscribe("mstr_cntl_brd_debug", 1000, mavric_debug_callback);

  ros::spin();

  return 0;
}
