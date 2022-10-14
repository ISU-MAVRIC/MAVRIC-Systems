from can.interface import Bus
from can import Message

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

    def init_controller(self, canID):
        """
        Initializes Spark Max controllers for sending and receiving messages for a specific controller.

        @param canID: ID of the controller
        @type canID: int
        """

        # TODO: Create function for initiating controller objects, saving them to the SparkBus object, and returning
        #  a pointer

        return None

    def send_msg(self, msg):
        """
        Sends msg to controllers via CAN bus.

        @param msg: CAN message to be sent to controller.
        @type msg: Message
        """

        # TODO: Create function for sending messages to the CAN bus, must include a CAN error catch
        #  (see CAN_Output_Test.py)

    def send_heartbeat(self):
        """
        Sends heartbeat message to CAN bus.
        """

        # TODO: Create function for sending heartbeat, should use the send_msg function (see CAN_Output_Test.py)

