from can.interface import Bus
from can import Message
from Statuses import Status
from SparkCAN import packer_float

"""
Description: Objects for storing data from motor controllers and sending data to motor controllers
Author: Jacob Peskuski, Gabriel Carlson
"""


class Controller:
    def __init__(self, bus, id):
        self.bus = bus
        self.id = id
        self.statuses = [Status(0x2051840+id, (32, 8, 12, 12), ('float', 'uint', 'uint', 'uint')),
                         Status(0x2051880+id, (32,), ('float',))]
        # TODO: Create value for storing properties of the motor controller and motor

    def enable(self):
        pass
        # TODO: Send enable control message to motor controller

    def disable(self):
        pass
        # TODO: Send disable control message to motor controller

    def percent_output(self, value):
        msg = can.Message(arbitration_id= 0x02050080 + id, data=packer_float(value))
        self.bus.send_msg(msg)

    def velocity_output(self, value):
        msg = can.Message(arbitration_id= 0x02050480 + id, data=packer_float(value))
        self.bus.send_msg(msg)

    @property
    def velocity(self):
        return None
        # TODO: Return velocity value from status message 1

    @property
    def position(self):
        return None
        # TODO: Return position value from status message 2

