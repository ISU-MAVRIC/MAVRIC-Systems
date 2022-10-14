from can.interface import Bus
from can import Message

"""
Description: Objects for decoding status messages sent from the motor controllers
Author: Jacob Peskuski, Gabriel Carlson
"""


class Status:
    def __init__(self, id, datasizes, datatypes):
        """
        Object for decoding Status messages from motor controllers.

        @param id: full CAN ID of the message.
        @type id: int
        @param datasizes: Sizes in bits of each of the data partitions of the message.
        @type datasizes: tuple
        @param datatypes: The datatypes each of the partitions represent (float, int, uint).
        @type: tuple
        """

        # TODO: Set up variables for object

    def decode(self, msg):
        """
        Decodes a CAN message into its values.

        @param msg: Message to be decoded.
        @type msg: Message
        """

        # TODO: Write function to convert CAN message to binary, seperate the bits, convert bits into values.

