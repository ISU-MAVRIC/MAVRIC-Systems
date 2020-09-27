#define Phoenix_No_WPI

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "mavric/Drivetrain.h"

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


double c_Scale = 0.4;
double c_lfDir = 1;
double c_lmDir = 1;
double c_lbDir = 1;
double c_rfDir = -1;
double c_rmDir = -1;
double c_rbDir = -1;

double leftTarget = 0;
double rightTarget = 0;

ros::Publisher lf_pub;
ros::Publisher lm_pub;
ros::Publisher lb_pub;
ros::Publisher rf_pub;
ros::Publisher rm_pub;
ros::Publisher rb_pub;

TalonSRX talon_lf(1);
TalonSRX talon_lm(2);
TalonSRX talon_lb(3);
TalonSRX talon_rf(4);
TalonSRX talon_rm(5);
TalonSRX talon_rb(6);


void driveCallback(const mavric::Drivetrain::ConstPtr &data)
{
	double dLeft = data->left;
	double dRight = data->right;

	if(dLeft > 100)
		dLeft = 100;
	if(dLeft < -100)
		dLeft = -100;

	if(dRight > 100)
		dRight = 100;
	if(dRight < -100)
		dRight = -100;

	leftTarget = dLeft / 100;
	rightTarget = dRight / 100;
}

void setOutputs(double lf, double lm, double lb, double rf, double rm, double rb)
{
	talon_lf.Set(ControlMode::PercentOutput, lf*c_Scale*c_lfDir);
	talon_lm.Set(ControlMode::PercentOutput, lm*c_Scale*c_lmDir);
	talon_lb.Set(ControlMode::PercentOutput, lb*c_Scale*c_lbDir);
	talon_rf.Set(ControlMode::PercentOutput, rf*c_Scale*c_rfDir);
	talon_rm.Set(ControlMode::PercentOutput, rm*c_Scale*c_rmDir);
	talon_rb.Set(ControlMode::PercentOutput, rb*c_Scale*c_rbDir);
}

double rampVal(double current, double target, double rampAmountUp, double rampAmountDown)
{
	if(current == target)
		return target;

	if(current >= 0 && target > 0)
	{
		if(current < target)
		{
			current += min(rampAmountUp, target-current);
		} else {
			current -= min(rampAmountDown, current-target);
		}
	} else if(current > 0 && target <= 0)
	{
		current -= min(rampAmountDown, current-target);
	} else if(current <= 0 && target < 0)
	{
		if(current > target)
		{
			current -= min(rampAmountUp, current-target);
		} else {
			current += min(rampAmountDown, target-current);
		}
	} else if(current < 0 && target >= 0)
	{
		current += min(rampAmountDown, target-current);
	} else {
		printf("case missed (%lf -> %lf)", current, target);
		current = target;
	}

	return current;
}

int main(int argc, char **argv)
{
	double left = 0;
	double right = 0;

	double rampRateUp = 0.5;
	double rampRateDown = 0.5;

	ros::init(argc, argv, "CAN_DTS");

	ctre::phoenix::platform::can::SetCANInterface("can0");
	//c_SetPhoenixDiagnosticsStartTime(-1);

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("Drive_Train", 1000, driveCallback);

	//ros::Service("SetProtection", SetBool, changeProtection);

	ros::param::get("~Range", c_Scale);
	ros::param::get("~Left_Front/Scale", c_lfDir);
	ros::param::get("~Left_Middle/Scale", c_lmDir);
	ros::param::get("~Left_Back/Scale", c_lbDir);
	ros::param::get("~Right_Front/Scale", c_rfDir);
	ros::param::get("~Right_Middle/Scale", c_rmDir);
	ros::param::get("~Right_Back/Scale", c_rbDir);
	ros::param::get("~ramp_rate_up", rampRateUp);
	ros::param::get("~ramp_rate_down", rampRateDown);

	setOutputs(0,0,0, 0,0,0);

	ros::Rate r(100);
	while(ros::ok())
	{
		ctre::phoenix::unmanaged::FeedEnable(200);
		ros::spinOnce();

		if(left != leftTarget || right != rightTarget)
		{
			left = rampVal(left, leftTarget, rampRateUp, rampRateDown);
			right = rampVal(right, rightTarget, rampRateUp, rampRateDown);
			setOutputs(left, left, left, right, right, right);
		}

		r.sleep();
	}

	return 0;
}
