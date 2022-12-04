import rospy

from mavric.msg import Steer, Drivetrain, Steertrain
from mavric.SparkCAN.SparkCAN import SparkBus



c_Scale = 60*2*3.14159/0.4167
c_lfDir = 1
c_lmDir = -1
c_lbDir = -1
c_rfDir = 1
c_rmDir = -1
c_rbDir = 1
c_str_Scale = 0.25
c_str_lfDir = 1
c_str_lbDir = -1
c_str_rfDir = 1
c_str_rbDir = -1
c_pitch = 0.2

lfTarget = 0
lmTarget = 0
lbTarget = 0
rfTarget = 0
rmTarget = 0
rbTarget = 0
strLfTarget = 0
strLbTarget = 0
strRfTarget = 0
strRbTarget = 0
pitchTarget = 0

dlf = 0
dlm = 0
dlb = 0
drf = 0
drm = 0
drb = 0
strLf = 0
strLb = 0
strRf = 0
strRb = 0
pitch = 0

rampRateUp = 0.5
rampRateDown = 0.5
strRateUp = 100
strRateDown = 100
pRateUp = 100
pRateDown = 100

sparkBus = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

spark_lf = sparkBus.init_controller(1)
spark_lm = sparkBus.init_controller(2)
spark_lb = sparkBus.init_controller(3)
spark_rf = sparkBus.init_controller(4)
spark_rm = sparkBus.init_controller(5)
spark_rb = sparkBus.init_controller(6)

spark_str_lf = sparkBus.init_controller(7)
spark_str_lb = sparkBus.init_controller(8)
spark_str_rf = sparkBus.init_controller(9)
spark_str_rb = sparkBus.init_controller(10)

#add publish and subs


def strpub():
  steerMsg = Steer()
  steerMsg.lf = spark_str_lf.position
  steerMsg.lb = spark_str_lb.position
  steerMsg.rf = spark_str_rf.position
  steerMsg.rb = spark_str_rb.position
  strPub.publish(steerMsg)
  

#Calibration KIA currently

def strCallback(data):
  slf = data.strLf
  if (slf > 100):
    slf = 100
  if (slf < -100):
    slf = -100

  slb = data.strLb
  if (slb > 100):
    slb = 100
  if (slb < -100):
    slb = -100

  srf = data.strRf
  if (srf > 100):
    srf = 100
  if (srf < -100):
    srf = -100

  srb = data.strRb
  if (srb > 100):
    srb = 100
  if (srb < -100):
    srb = -100


def driveCallback(data):
	lf = data.lf
	lm = data.lm
	lb = data.lb
	rf = data.rf
	rm = data.rm
	rb = data.rb

	if (lf > 100):
		lf = 100
	if (lf < -100):
		lf = -100

	if (lm > 100):
		lm = 100
	if (lm < -100):
		lm = -100

	if (lb > 100):
		lb = 100
	if (lb < -100):
		lb = -100

	if (rf > 100):
		rf = 100
	if (rf < -100):
		rf = -100

	if (rm > 100):
		rm = 100
	if (rm < -100):
		rm = -100

	if (rb > 100):
		rb = 100
	if (rb < -100):
		rb = -100




def setOutputs(lf, lm, lb, rf, rm, rb, str_lf, str_lb, str_rf, str_rb):
	spark_lf.velocity_output(lf * c_Scale * c_lfDir)
	spark_lm.velocity_output(lm * c_Scale * c_lmDir)
	spark_lb.velocity_output(lb * c_Scale * c_lbDir)
	spark_rf.velocity_output(rf * c_Scale * c_rfDir)
	spark_rm.velocity_output(rm * c_Scale * c_rmDir)
	spark_rb.velocity_output(rb * c_Scale * c_rbDir)

	talon_str_lf.position_output(str_lf * c_str_lfDir * c_str_Scale)
	talon_str_lb.position_output(str_lb * c_str_lbDir * c_str_Scale)
	talon_str_rf.position_output(str_rf * c_str_rfDir * c_str_Scale)
	talon_str_rb.position_output(str_rb * c_str_rbDir * c_str_Scale)


def main():
    rospy.init_node("CAN_DTS")

    sub = rospy.Subscriber("Drive_Train", Drivetrain, driveCallback, queue_size = 10)
    str_sub = rospy.Subscriber("Steer_Train", Steertrain, strCallback, queue_size = 10)
    #cal_sub = rospy.Subscriber("Steer_Cal", 1000, CalCallback);
    str_pub = rospy.Publisher("Steer_Feedback", Steer, queue_size=10)

    c_Scale = rospy.get_param("~Range")
    c_str_Scale = rospy.get_param("~Str/Range")
    c_lfDir = rospy.get_param("~Left_Front/Scale")
    c_lmDir = rospy.get_param("~Left_Middle/Scale")
    c_lbDir = rospy.get_param("~Left_Back/Scale")
    c_rfDir = rospy.get_param("~Right_Front/Scale")
    c_rmDir = rospy.get_param("~Right_Middle/Scale")
    c_rbDir = rospy.get_param("~Right_Back/Scale")
    c_pitch = rospy.get_param("~Pitch/Scale")
    rampRateUp = rospy.get_param("~ramp_rate_up")
    rampRateDown = rospy.get_param("~ramp_rate_down")
    strRateUp = rospy.get_param("~str_ramp_rate_up")
    strRateDown = rospy.get_param("~str_ramp_rate_down")

    setOutputs(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    rosRate = rospy.Rate(30)
    while not rospy.is_shutdown():
        setOutputs(dlf, dlm, dlb, drf, drm, drb, strLf, strLb, strRf, strRb)
        strpub()
        r.sleep()