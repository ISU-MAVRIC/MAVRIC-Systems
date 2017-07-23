#include "ros/ros.h"
#include "std_msgs/Byte.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "external");
  ros::NodeHandle thisNode;
  ros::Publisher publishers[9];
  int autoMode = 0;
  {
    ros::NodeHandle privateNode("~");
    privateNode.getParam("autoMode", autoMode);
  }
  int i;
  for (i = 0; i < 9; i++) {
    char* topicName = (char*)malloc(50);
    sprintf(topicName, "mstr_cntl_brd/debug/gpio/%d", i+2);
    ROS_INFO("%s", topicName);
    publishers[i] = thisNode.advertise<std_msgs::Byte>(topicName, 1000);
  }
  ros::Rate loop_rate(10);

  char states[9] = {0};
  while (ros::ok()) {
    std_msgs::Byte msg;
    
    int index;
    if (autoMode == 0) {
      scanf("%d", &index);
    } else {
      index = autoMode;
    }
    if (index == 0) {
      msg.data = 0;
      for (i = 0; i < 9; i++) {
	states[i] = 0;
	publishers[i].publish(msg);
	ROS_INFO("clearing %d", i);
      }
    } else {
      index--;
      states[index] ^= 1;
      msg.data = states[index];
      publishers[index].publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
