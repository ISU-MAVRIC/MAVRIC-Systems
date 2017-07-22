#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char* argv) {
  ros::init(argc, argv, "external");
  ros::NodeHandle thisNode;
  ros::Publisher publisher = thisNode.advertise<std::msgs::String>("external-debug", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << count;
    msg.data = ss.str();
    publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
}
