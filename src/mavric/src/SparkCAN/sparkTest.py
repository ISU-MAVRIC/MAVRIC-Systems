from SparkCAN import SparkBus

import time

busy = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)
sparkESC = busy.init_controller(2)
time.sleep(5)
sparkESC.velocity_output(1000)
time.sleep(5)
sparkESC.percent_output(0)
