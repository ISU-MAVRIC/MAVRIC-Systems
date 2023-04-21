#define Phoenix_No_WPI

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
//#include "mavric/Steer.h"
#include "mavric/Drivetrain.h"
//#include "mavric/Steertrain.h"
//#include "mavric/Steercal.h"

#include <algorithm>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ros;
using namespace std_msgs;

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::unmanaged;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


double c_spDir = 1;
double c_srDir = 1;
double c_epDir = 1;
double c_wpDir = 1;
double c_wrDir = 1;


double c_sr = 0.2;
double c_ep = 0.2;
double c_wr = 0.2;
double c_wp = 0.2;



double srTarget = 0;
double epTarget = 0;
double wrTarget = 0;
double wpTarget = 0;

double sr = 0;
double ep = 0;
double wr = 0;
double wp = 0;

double rampRateUp = 0.5;
double rampRateDown = 0.5;
double srRateUp = 100;
double srRateDown = 100;
double epRateUp = 100;
double epRateDown = 100;
double wrRateUp = 100;
double wrRateDown = 100;
double wpRateUp = 100;
double wpRateDown = 100;

TalonSRX talon_sr(1);
TalonSRX talon_ep(2);
TalonSRX talon_wr(3);
TalonSRX talon_wp(5);


void srCallback(const std_msgs::Float64::ConstPtr &data)
{
	double val = data->data;

	if (val > 100)
		val = 100;
	if (val < -100)
		val = -100;

	srTarget = val/100;
}

void epCallback(const std_msgs::Float64::ConstPtr &data)
{
	double val = data->data;

	if (val > 100)
		val = 100;
	if (val < -100)
		val = -100;

	epTarget = val/100;
}

void wrCallback(const std_msgs::Float64::ConstPtr &data)
{
	double val = data->data;

	if (val > 100)
		val = 100;
	if (val < -100)
		val= -100;

	wrTarget = val/100;
}

void wpCallback(const std_msgs::Float64::ConstPtr &data)
{
	double val = data->data;

	if (val > 100)
		val = 100;
	if (val < -100)
		val = -100;

	wpTarget = val/100;
}


double rampVal(double current, double target, double rampAmountUp, double rampAmountDown)
{
	if (current == target)
		return target;

	if (current >= 0 && target > 0)
	{
		if (current < target)
		{
			current += min(rampAmountUp, target - current);
		}
		else
		{
			current -= min(rampAmountDown, current - target);
		}
	}
	else if (current > 0 && target <= 0)
	{
		current -= min(rampAmountDown, current - target);
	}
	else if (current <= 0 && target < 0)
	{
		if (current > target)
		{
			current -= min(rampAmountUp, current - target);
		}
		else
		{
			current += min(rampAmountDown, target - current);
		}
	}
	else if (current < 0 && target >= 0)
	{
		current += min(rampAmountDown, target - current);
	}
	else
	{
		printf("case missed (%lf -> %lf)", current, target);
		current = target;
	}

	return current;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "CAN_ATS");

	ctre::phoenix::platform::can::SetCANInterface("can0");
	//c_SetPhoenixDiagnosticsStartTime(-1);

	ros::NodeHandle n;
	ros::Subscriber sr_sub = n.subscribe("SR_Train", 1000, srCallback);
    ros::Subscriber ep_sub = n.subscribe("EP_Train", 1000, epCallback);
    ros::Subscriber wr_sub = n.subscribe("WR_Train", 1000, wrCallback);
    ros::Subscriber wp_sub = n.subscribe("WP_Train", 1000, wpCallback);

	//ros::Service("SetProtection", SetBool, changeProtection);

	ros::param::get("~sr/Scale", c_sr);
	ros::param::get("~sr_ramp_rate_up", srRateUp);
	ros::param::get("~sr_ramp_rate_down", srRateDown);
    ros::param::get("~ep/Scale", c_ep);
	ros::param::get("~ep_ramp_rate_up", epRateUp);
	ros::param::get("~ep_ramp_rate_down", epRateDown);
    ros::param::get("~wr/Scale", c_wr);
	ros::param::get("~wr_ramp_rate_up", wrRateUp);
	ros::param::get("~wr_ramp_rate_down", wrRateDown);
    ros::param::get("~wp/Scale", c_wp);
	ros::param::get("~wp_ramp_rate_up", wpRateUp);
	ros::param::get("~wp_ramp_rate_down", wpRateDown);



	ros::Rate r(30);
	while (ros::ok())
	{
		ctre::phoenix::unmanaged::FeedEnable(200);
		ros::spinOnce();



    if (sr != srTarget)
		{
			sr = rampVal(sr, srTarget, srRateUp, srRateDown);
			talon_sr.Set(ControlMode::PercentOutput, sr * c_sr * c_srDir);
		}

    if (ep != epTarget)
		{
			ep = rampVal(ep, epTarget, epRateUp, epRateDown);
			talon_ep.Set(ControlMode::PercentOutput, ep * c_ep * c_epDir);
		}

    if (wr != wrTarget)
		{
			wr = rampVal(wr, wrTarget, wrRateUp, wrRateDown);
			talon_wr.Set(ControlMode::PercentOutput, wr * c_wr * c_wrDir);
		}

    if (wp != wpTarget)
		{
			wp = rampVal(wp, wpTarget, wpRateUp, wpRateDown);
			talon_wp.Set(ControlMode::PercentOutput, wp * c_wp * c_wpDir);
		}


		r.sleep();
	}

	return 0;
}