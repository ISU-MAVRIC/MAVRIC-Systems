#define Phoenix_No_WPI

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ros;
using namespace std_msgs;

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::unmanaged;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonSRX talon(7);

void tstCallback(const std_msgs::Float64::ConstPtr &msg)
{
	printf("rx msg\n");
	talon.Set(ControlMode::PercentOutput, msg->data);
	//ROS_INFO("Heard: %lf", msg->data);
	printf("%lf\n", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CAN_DTS");

	ctre::phoenix::platform::can::SetCANInterface("can0");
	//c_SetPhoenixDiagnosticsStartTime(-1);

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/CAN/tstNodeTopic", 1000, tstCallback);

	printf("Im startin lol\n");

	ros::Rate r(100);
	while(ros::ok())
	{
		ctre::phoenix::unmanaged::FeedEnable(100);
		//talon.Set(ControlMode::PercentOutput, val);
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
