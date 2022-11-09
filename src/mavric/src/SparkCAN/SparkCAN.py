from can.interface import Bus
from can import Message, CanError
from threading import Thread
import time

from SparkController import Controller


"""
Description: Library for providing objects for controlling and receving feedback from multiple Spark Max Controllers via
CAN.
Author: Jacob Peskuski, Gabriel Carlson
"""


class SparkBus:
    def __init__(self, channel='can0', bustype='socketcan', bitrate=1000000):
        """
        Object for sending and receiving Spark Max CAN messages.

        @param channel: Serial channel the CAN interface is on.
        @type channel: str
        @param bustype: Type of bus, set to 'None' for to let it be resolved automatically from the default
        configuration.
        @type bustype: str
        @param bitrate: Rate at which bits are sent through the CAN bus.
        @type bitrate: int
        """
        # init CAN bus
        self.bus = Bus(channel=channel, bustype=bustype, bitrate=bitrate)

        self.controllers = {}

        self.can_ids = []

        # Start heartbeat thread
        self.heartbeat_enabled = True
        self.enable_id_array = [0, 0, 0, 0, 0, 0, 0, 0]
        self.heartbeat_thread = Thread(target=self._heartbeat_runnable, daemon=True)
        self.heartbeat_thread.start()

        # Start monitor thread
        self.monitor_thread = Thread(target=self.bus_monitor, daemon=True)
        self.monitor_thread.start()

    def init_controller(self, canID):
        """
        Initializes Spark Max controllers for sending and receiving messages for a specific controller.

        @param canID: ID of the controller
        @type canID: int
        @return: Controller object pointer
        @rtype: Controller
        """

        # create new controller object, add it to list of controllers
        self.controllers.update({canID: Controller(self, canID)})

        self.can_ids.append(canID)

        # update enable_id_array
        self._update_heartbeat_array()

        return self.controllers.get(canID)

    def send_msg(self, msg):
        """
        Sends msg to controllers via CAN bus.

        @param msg: CAN message to be sent to controller.
        @type msg: Message
        """
        try:
            self.bus.send(msg)
        except CanError as err:
            print(err)

    def bus_monitor(self):
        """
        Thread for monitoring the bus for receivable messages.
        """

        while True:
            message = self.bus.recv(0)
            if message is None:
                time.sleep(0.002)
                continue

            # get api (class and index) and id of device from the message id
            api = (message.arbitration_id & 0x0000FFC0) >> 6
            devID = (message.arbitration_id & 0x0000003F)

            if devID in self.controllers.keys() and api in self.controllers[devID].statuses.keys() and self.controllers[devID].statuses[api] is not None:
                # using device id and api, send message to decoder
                self.controllers[devID].statuses[api].decode(message.data)

    def enable_heartbeat(self):
        """
        Enables heartbeat runnable for sending heartbeat message to CAN Bus
        """
        self.heartbeat_enabled = True

    def disable_heartbeat(self):
        """
        Disables heartbeat runnable for sending heartbeat message to CAN Bus
        """
        self.heartbeat_enabled = False

    def _update_heartbeat_array(self):
        enable_array = ['0'] * 64
        for id in self.can_ids:
            enable_array[id] = '1'
        enable_array.reverse()
        self.enable_id_array = [0, 0, 0, 0, 0, 0, 0, 0]
        self.enable_id_array[7] = int("".join(enable_array[0:8]), 2)
        self.enable_id_array[6] = int("".join(enable_array[8:16]), 2)
        self.enable_id_array[5] = int("".join(enable_array[16:24]), 2)
        self.enable_id_array[4] = int("".join(enable_array[24:32]), 2)
        self.enable_id_array[3] = int("".join(enable_array[32:40]), 2)
        self.enable_id_array[2] = int("".join(enable_array[40:48]), 2)
        self.enable_id_array[1] = int("".join(enable_array[48:56]), 2)
        self.enable_id_array[0] = int("".join(enable_array[56:64]), 2)

    def _heartbeat_runnable(self):
        while True:
            if self.heartbeat_enabled:
                # set when init_controller is called
                msg = Message(arbitration_id=0x02052480, data=self.enable_id_array)
                self.send_msg(msg)
                time.sleep(.002)
