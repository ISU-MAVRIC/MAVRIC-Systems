from can.interface import Bus
from can import Message
from Statuses import Status
from struct import pack

"""
Description: Objects for storing data from motor controllers and sending data to motor controllers
Author: Jacob Peskuski, Gabriel Carlson
"""


def packer_float(value):
    hexStr = pack('fi', value, 0)
    return hexStr


class Controller:
    def __init__(self, bus, id):
        self.bus = bus
        self.id = id
        self.statuses = {0x60: None,
                         0x61: Status(0x2051840+id, (32, 8, 12, 12), ('float', 'uint', 'uint', 'uint')),
                         0x62: Status(0x2051880+id, (32,), ('float',)),
                         0x63: None,
                         0x64: None}

        # control properties
        self.percentProps = {"dir": 1, "scale": 1}
        self.velocityProps = {"dir": 1, "countConversion": 1}

    def enable(self):
        pass
        # TODO: Send enable control message to motor controller

    def disable(self):
        pass
        # TODO: Send disable control message to motor controller

    def percent_output(self, value):
        mod_value = value*self.percentProps["dir"]*self.percentProps["scale"]
        msg = Message(arbitration_id= 0x02050080 + self.id, data=packer_float(mod_value))
        self.bus.send_msg(msg)

    def velocity_output(self, value):
        mod_value = value * self.velocityProps["dir"]*self.velocityProps["countConversion"]
        msg = Message(arbitration_id= 0x02050480 + self.id, data=packer_float(mod_value))
        self.bus.send_msg(msg)

    @property
    def velocity(self):
        return self.statuses[0x61].data[0]

    @property
    def position(self):
        return self.statuses[0x62].data[0]
