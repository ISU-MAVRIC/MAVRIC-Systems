from can.interface import Bus
from can import Message
from threading import Thread
import time


"""
Description: Library for providing objects for controlling and receving feedback from multiple Spark Max Controllers via
CAN.
Author: Jacob Peskuski, Gabriel Carlson
"""


class SparkBus(Bus):
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
        Bus.__init__(self, channel=channel, bustype=bustype, bitrate=bitrate)

        
        self.can_ids = []


        #Start heartbeat thread
        self.heartbeat_enabled = True
        self.enable_id_array = [0, 0, 0, 0, 0, 0, 0, 0]
        self.heartbeat_thread = Thread(target=_heartbeat_runnable, daemon=True)
        self.heartbeat_thread.start()

    def init_controller(self, canID):
        """
        Initializes Spark Max controllers for sending and receiving messages for a specific controller.

        @param canID: ID of the controller
        @type canID: int
        """

        # TODO: Create function for initiating controller objects, saving them to the SparkBus object, and returning
        #  a pointer

        self.can_ids.append(canID)

        #update enable_id_array
        _update_heartbeat_array()

        return None

    def send_msg(self, msg):
        """
        Sends msg to controllers via CAN bus.

        @param msg: CAN message to be sent to controller.
        @type msg: Message
        """
        try:
            self.send(msg)
        except can.CanError as err:
            print(err)

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
        self.enable_id_array[7] = hex(int("".join(enable_array[0:8]), 2))
        self.enable_id_array[6] = hex(int("".join(enable_array[8:16]), 2))
        self.enable_id_array[5] = hex(int("".join(enable_array[16:24]), 2))
        self.enable_id_array[4] = hex(int("".join(enable_array[24:32]), 2))
        self.enable_id_array[3] = hex(int("".join(enable_array[32:40]), 2))
        self.enable_id_array[2] = hex(int("".join(enable_array[40:48]), 2))
        self.enable_id_array[1] = hex(int("".join(enable_array[48:56]), 2))
        self.enable_id_array[0] = hex(int("".join(enable_array[56:64]), 2))

    def _heartbeat_runnable(self):
        while True:
            if self.heartbeat_enabled:
                msg = can.Message(arbitration_id=0x02052480,
                data=self.enable_id_array)  # set when init_controller is called
                self.send_msg(msg)
                time.sleep(.002)