#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gpio.h"

#define PIN 2

void mavric_debug_callback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Message Recieved: %s", msg->data.c_str());
  static int state = 0;
  GPIOWrite(PIN, state);
  state ^= 1;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mstr_cntl_brd_debug");
  ros::NodeHandle thisNode;

  GPIOExport(PIN);
  ros::Duration(1).sleep();
  GPIODirection(PIN, OUT);
  GPIOWrite(PIN, 0);
  
  ros::Subscriber subscription = thisNode.subscribe("mstr_cntl_brd_debug", 1000, mavric_debug_callback);

  ros::spin();

  GPIOUnexport(PIN);

  return 0;
}
