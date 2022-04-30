#include <iostream>		
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <algorithm>

using namespace ros;
using namespace std_msgs;
using namespace std;

double counter = 0;
double dir = 1;
int ch1 = 28;
int ch2 = 29;

void addCount(){
    if(digitalRead(ch2) == 0) {
        counter += 1;
    }
    else {
        counter += -1;
    }

}

//void dirForward(){
    //dir = 1;
    //ROS_INFO("%s", "f");
//}

//void dirBackward(){
    //dir = -1;
    //ROS_INFO("%s", "b");
//}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Encoder_FB");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64>("Motor_Feedback", 10);
    ros::param::get("~ch1", ch1);
    ros::param::get("~ch2", ch2);
    std_msgs::Float64 pub_data;
    ros::Rate loop_rate(30);

    wiringPiSetup();
    pinMode(ch1, INPUT);
    pinMode(ch2, INPUT);
    wiringPiISR(ch1, INT_EDGE_RISING, addCount);
    //wiringPiISR(ch2, INT_EDGE_RISING, dirBackward);
    //wiringPiISR(ch2, INT_EDGE_FALLING, dirForward);

	while (ros::ok()){
        pub_data.data = counter;
        pub.publish(pub_data);
        loop_rate.sleep();
    }

    return 0;
}