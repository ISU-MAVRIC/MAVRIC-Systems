from SparkCAN import SparkBus

import time

busy = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)
sparkESC = busy.init_controller(3)
sparkESC2 = busy.init_controller(2)
time.sleep(1)
sparkESC.percent_output(2)
for i in range(5000):
    print(sparkESC.velocity)
    print(sparkESC2.velocity)
    time.sleep(0.001)
sparkESC.percent_output(0)

sparkESC2.percent_output(1)
for i in range(5000):
    print(sparkESC.velocity)
    print(sparkESC2.velocity)
    time.sleep(0.001)

sparkESC.percent_output(0)
sparkESC2.percent_output(0)
