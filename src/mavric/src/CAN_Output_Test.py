import can
from time import time, sleep
from struct import *
import math

bus = can.interface.Bus('can0', bustype='socketcan', bitrate=1000000)


def can_send(msg):
    try:
        bus.send(msg)
        #print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message not sent")


def send_heartbeat():
    msg = can.Message(arbitration_id=0x02052480,
            data=[0x06, 0, 0, 0, 0, 0, 0, 0])  # mask enabling ids 1 + 2 ...  0x0e enables ids 1,2, and 3
    can_send(msg)


def packer_float(value):
    hexStr = pack('fi', value, 0)
    return hexStr


def set_percent_output(id, value):
    msg = can.Message(arbitration_id= 0x02050080 + id, data=packer_float(value))
    can_send(msg)


def set_velocity_output(id, value):
    msg = can.Message(arbitration_id= 0x02050480 + id, data=packer_float(value))
    can_send(msg)


while True:
    send_heartbeat()
    set_percent_output(2, math.sin(time()*0.25))
    sleep(0.002)


# while True:
#     send_heartbeat()
#     set_velocity_output(2, 100)
#     sleep(0.002)