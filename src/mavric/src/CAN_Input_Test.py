import can
from time import time, sleep
from struct import *
import math
import bitstring

bus = can.interface.Bus('can0', bustype='socketcan', bitrate=1000000)


# Object for storing data from periodic status message 1
class Status1:
    def __init__(self, id):
        self.id = id
        self.msgSizes = (0, 32)
        self.data = [0]

    def new_value(self, msg):
        data = bitstring.BitArray(msg)
        for i in range(1, len(data)):
            bits = data.bin[i-1, i]
            sub = bitstring.Bits(bin=bits)
            self.data[i-1] = sub.float


s1 = Status1(0x2051842)
while True:
    message = bus.recv(0)
    if message is not None:
        if message.arbitration_id == 0x2051842:
            s1.new_value(message.data)
            print(s1.data)
    sleep(0.002)