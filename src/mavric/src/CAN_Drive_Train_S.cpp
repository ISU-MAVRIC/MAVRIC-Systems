#define Phoenix_No_WPI

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "mavric/Steer.h"
#include "mavric/Drivetrain.h"
#include "mavric/Steertrain.h"
#include "mavric/Steercal.h"

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

double c_Scale = 0.3;
double c_lfDir = 1;
double c_lmDir = -1;
double c_lbDir = 1;
double c_rfDir = -1;
double c_rmDir = -1;
double c_rbDir = -1;
double c_str_Scale = 227.55;
double c_str_lfDir = 1;
double c_str_lbDir = -1;
double c_str_rfDir = 1;
double c_str_rbDir = -1;

double lfTarget = 0;
double lmTarget = 0;
double lbTarget = 0;
double rfTarget = 0;
double rmTarget = 0;
double rbTarget = 0;
double strLfTarget = 0;
double strLbTarget = 0;
double strRfTarget = 0;
double strRbTarget = 0;

TalonSRX talon_lf(1);
TalonSRX talon_lm(2);
TalonSRX talon_lb(3);
TalonSRX talon_rf(4);
TalonSRX talon_rm(5);
TalonSRX talon_rb(6);

TalonSRX talon_str_lf(7);
TalonSRX talon_str_lb(8);
TalonSRX talon_str_rf(9);
TalonSRX talon_str_rb(10);

ErrorCode sen1 = talon_str_lf.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
ErrorCode sen2 = talon_str_lb.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
ErrorCode sen3 = talon_str_rf.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
ErrorCode sen4 = talon_str_rb.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

//ErrorCode cur1 = talon_str_lf.ConfigPeakCurrentLimit(7, 0);
//ErrorCode cur2 = talon_str_lb.ConfigPeakCurrentLimit(7, 0);
//ErrorCode cur3 = talon_str_rf.ConfigPeakCurrentLimit(7, 0);
//ErrorCode cur4 = talon_str_rb.ConfigPeakCurrentLimit(7, 0);

SensorCollection lf_FB = talon_str_lf.GetSensorCollection();
SensorCollection lb_FB = talon_str_lb.GetSensorCollection();
SensorCollection rf_FB = talon_str_rf.GetSensorCollection();
SensorCollection rb_FB = talon_str_rb.GetSensorCollection();

ErrorCode cal1 = lf_FB.SetQuadraturePosition(0, 0);
ErrorCode cal2 = lb_FB.SetQuadraturePosition(0, 0);
ErrorCode cal3 = rf_FB.SetQuadraturePosition(0, 0);
ErrorCode cal4 = rb_FB.SetQuadraturePosition(0, 0);

void strpub(const ros::Publisher pub)
{
	mavric::Steer value;
	value.lf = lf_FB.GetQuadraturePosition();
	value.lb = lb_FB.GetQuadraturePosition();
	value.rf = rf_FB.GetQuadraturePosition();
	value.rb = rb_FB.GetQuadraturePosition();
	pub.publish(value);
}

void CalCallback(const mavric::Steercal::ConstPtr &data)
{
	if (data->calLf == true)
		ErrorCode cal1 = lf_FB.SetQuadraturePosition(0, 0);
	if (data->calLb == true)
		ErrorCode cal2 = lb_FB.SetQuadraturePosition(0, 0);
	if (data->calRf == true)
		ErrorCode cal3 = rf_FB.SetQuadraturePosition(0, 0);
	if (data->calRb == true)
		ErrorCode cal4 = rb_FB.SetQuadraturePosition(0, 0);
}

void strCallback(const mavric::Steertrain::ConstPtr &data)
{
	double slf = data->strLf;
	if (slf > 100)
		slf = 100;
	if (slf < -100)
		slf = -100;
	strLfTarget = (int)slf;

	double slb = data->strLb;
	if (slb > 100)
		slb = 100;
	if (slb < -100)
		slb = -100;
	strLbTarget = (int)slb;

	double srf = data->strRf;
	if (srf > 100)
		srf = 100;
	if (srf < -100)
		srf = -100;
	strRfTarget = (int)srf;

	double srb = data->strRb;
	if (srb > 100)
		srb = 100;
	if (srb < -100)
		srb = -100;
	strRbTarget = (int)srb;
}

void driveCallback(const mavric::Drivetrain::ConstPtr &data)
{
	double lf = data->lf;
	double lm = data->lm;
	double lb = data->lb;
	double rf = data->rf;
	double rm = data->rm;
	double rb = data->rb;

	if (lf > 100)
		lf = 100;
	if (lf < -100)
		lf = -100;

	if (lm > 100)
		lm = 100;
	if (lm < -100)
		lm = -100;
	
	if (lb > 100)
		lb = 100;
	if (lb < -100)
		lb = -100;

	if (rf > 100)
		rf = 100;
	if (rf < -100)
		rf = -100;

	if (rm > 100)
		rm = 100;
	if (rm < -100)
		rm = -100;

	if (rb > 100)
		rb = 100;
	if (rb < -100)
		rb = -100;

	lfTarget = lf / 100;
	lmTarget = lm / 100;
	lbTarget = lb / 100;
	rfTarget = rf / 100;
	rmTarget = rm / 100;
	rbTarget = rb / 100;
}

void setOutputs(double lf, double lm, double lb, double rf, double rm, double rb, double str_lf, double str_lb, double str_rf, double str_rb)
{
	talon_lf.Set(ControlMode::PercentOutput, lf * c_Scale * c_lfDir);
	talon_lm.Set(ControlMode::PercentOutput, lm * c_Scale * c_lmDir);
	talon_lb.Set(ControlMode::PercentOutput, lb * c_Scale * c_lbDir);
	talon_rf.Set(ControlMode::PercentOutput, rf * c_Scale * c_rfDir);
	talon_rm.Set(ControlMode::PercentOutput, rm * c_Scale * c_rmDir);
	talon_rb.Set(ControlMode::PercentOutput, rb * c_Scale * c_rbDir);

	talon_str_lf.Set(ControlMode::Position, str_lf * c_str_lfDir * c_str_Scale);
	talon_str_lb.Set(ControlMode::Position, str_lb * c_str_lbDir * c_str_Scale);
	talon_str_rf.Set(ControlMode::Position, str_rf * c_str_rfDir * c_str_Scale);
	talon_str_rb.Set(ControlMode::Position, str_rb * c_str_rbDir * c_str_Scale);
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
	double dlf = 0;
	double dlm = 0;
	double dlb = 0;
	double drf = 0;
	double drm = 0;
	double drb = 0;
	double strLf = 0;
	double strLb = 0;
	double strRf = 0;
	double strRb = 0;
	
	double rampRateUp = 0.5;
	double rampRateDown = 0.5;
	double strRateUp = 100;
	double strRateDown = 100;

	ros::init(argc, argv, "CAN_DTS");

	ctre::phoenix::platform::can::SetCANInterface("can0");
	//c_SetPhoenixDiagnosticsStartTime(-1);

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("Drive_Train", 1000, driveCallback);
	ros::Subscriber str_sub = n.subscribe("Steer_Train", 1000, strCallback);
	ros::Subscriber cal_sub = n.subscribe("Steer_Cal", 1000, CalCallback);
	ros::Publisher str_pub = n.advertise<mavric::Steer>("Steer_Feedback", 1000);

	//ros::Service("SetProtection", SetBool, changeProtection);

	ros::param::get("~Range", c_Scale);
	ros::param::get("~Str/Range", c_str_Scale);
	ros::param::get("~Left_Front/Scale", c_lfDir);
	ros::param::get("~Left_Middle/Scale", c_lmDir);
	ros::param::get("~Left_Back/Scale", c_lbDir);
	ros::param::get("~Right_Front/Scale", c_rfDir);
	ros::param::get("~Right_Middle/Scale", c_rmDir);
	ros::param::get("~Right_Back/Scale", c_rbDir);
	ros::param::get("~ramp_rate_up", rampRateUp);
	ros::param::get("~ramp_rate_down", rampRateDown);
	ros::param::get("~str_ramp_rate_up", strRateUp);
	ros::param::get("~str_ramp_rate_down", strRateDown);

	setOutputs(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	ros::Rate r(100);
	while (ros::ok())
	{
		ctre::phoenix::unmanaged::FeedEnable(200);
		ros::spinOnce();

		if (dlf != lfTarget || dlm != lmTarget ||dlb != lbTarget || drf != rfTarget ||  drm != rmTarget || drb != rbTarget || strLf != strLfTarget || strRf != strRfTarget || strLb != strLbTarget || strRb != strRbTarget)
		{
			dlf = rampVal(dlf, lfTarget, rampRateUp, rampRateDown);
			dlm = rampVal(dlm, lmTarget, rampRateUp, rampRateDown);
			dlb = rampVal(dlb, lbTarget, rampRateUp, rampRateDown);
			drf = rampVal(drf, rfTarget, rampRateUp, rampRateDown);
			drm = rampVal(drm, rmTarget, rampRateUp, rampRateDown);
			drb = rampVal(drb, rbTarget, rampRateUp, rampRateDown);
			strLf = rampVal(strLf, strLfTarget, strRateUp, strRateDown);
			strLb = rampVal(strLb, strLbTarget, strRateUp, strRateDown);
			strRf = rampVal(strRf, strRfTarget, strRateUp, strRateDown);
			strRb = rampVal(strRb, strRbTarget, strRateUp, strRateDown);
			setOutputs(dlf, dlm, dlb, drf, drm, drb, strLf, strLb, strRf, strRb);
		}

		strpub(str_pub);
		r.sleep();
	}

	return 0;
}
