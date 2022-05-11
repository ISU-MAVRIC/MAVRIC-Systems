#include <iostream>		
#include <JetsonGPIO.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <algorithm>

using namespace ros;
using namespace std_msgs;
using namespace std;
using namespace GPIO;

double counter = 0;
double dir = 1;
int ch1 = 38;
int ch2 = 40;

void addCount(){
    if(input(ch2) == 0) {
        counter += 1;
    }
    else {
        counter += -1;
    }

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Encoder_FB");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64>("Motor_Feedback", 10);
    ros::param::get("~ch1", ch1);
    ros::param::get("~ch2", ch2);
    std_msgs::Float64 pub_data;
    ros::Rate loop_rate(10);

    setmode(GPIO::BOARD);

    setup(ch1, IN);
    setup(ch2, IN);
    add_event_detect(ch1, RISING, addCount);

	while (ros::ok()){
        pub_data.data = counter;
        pub.publish(pub_data);
        loop_rate.sleep();
    }

    return 0;
}