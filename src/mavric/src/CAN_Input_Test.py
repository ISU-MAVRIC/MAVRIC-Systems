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
        self.msgSizes = (0, 32, 40, 52, 64)
        self.data = [0, 0, 0, 0]

    def new_value(self, msg):
        data = bitstring.BitArray(msg)
        bits = data.bin[self.msgSizes[0]:self.msgSizes[1]]
        sub = bitstring.Bits(bin=bits)
        self.data[0] = sub.floatle
        bits = data.bin[self.msgSizes[1]:self.msgSizes[2]]
        sub = bitstring.Bits(bin=bits)
        self.data[1] = sub.uintle
        bits = data.bin[self.msgSizes[2]:self.msgSizes[3]]
        sub = bitstring.Bits(bin=bits)
        self.data[2] = sub.int
        bits = data.bin[self.msgSizes[3]:self.msgSizes[4]]
        sub = bitstring.Bits(bin=bits)
        self.data[3] = sub.uint


# Object for storing data from periodic status message 2
class Status2:
    def __init__(self, id):
        self.id = id
        self.msgSizes = (0, 32)
        self.data = [0]

    def new_value(self, msg):
        data = bitstring.BitArray(msg)
        bits = data.bin[self.msgSizes[0]:self.msgSizes[1]]
        sub = bitstring.Bits(bin=bits)
        self.data[0] = sub.floatle



s1 = Status1(0x2051842)
s2 = Status2(0x2051882)
while True:
    message = bus.recv(0)
    if message is not None:
        if message.arbitration_id == 0x2051842:
            s1.new_value(message.data)
            print(s1.data)
        if message.arbitration_id == 0x2051882:
            s2.new_value(message.data)
            print(s2.data)
    sleep(0.002)
