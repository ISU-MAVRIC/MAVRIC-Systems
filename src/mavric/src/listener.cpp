#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "gpio.h"

int PIN = 2;

void mavric_debug_callback(const std_msgs::Byte::ConstPtr& msg) {
  ROS_INFO("Message Recieved: %d", msg->data);
  GPIOWrite(PIN, msg->data);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mstr_cntl_brd_debug");
  ros::NodeHandle thisNode;

  {
    ros::NodeHandle privateNode("~");  
    privateNode.getParam("gpio_pin", PIN);
  }
  
  GPIOExport(PIN);
  ros::Duration(1).sleep();
  GPIODirection(PIN, OUT);
  GPIOWrite(PIN, 0);
  char topicName[100];
  sprintf(topicName, "mstr_cntl_brd/debug/gpio/%d", PIN);
  ros::Subscriber subscription = thisNode.subscribe(topicName, 1000, mavric_debug_callback);
  ROS_INFO("%s", topicName);
  ros::spin();

  GPIOUnexport(PIN);

  return 0;
}
