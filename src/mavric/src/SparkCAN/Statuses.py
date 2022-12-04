from can.interface import Bus
from can import Message
import bitstring

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

        self.id = id
        self.datasizes = (0,) + datasizes
        self.datatypes = datatypes
        self.data = [0]*len(datatypes)

    def decode(self, msg):
        """
        Decodes a CAN message into its values.

        @param msg: Message to be decoded.
        @type msg: Message
        """

        # loop through each data range to get each value in status message
        for i in range(len(self.datatypes)):
            # get start and end indexes of each value in the message
            start = sum(list(self.datasizes[0:i+1]))
            end = sum(list(self.datasizes[0:i+2]))

            # get string of bits
            data = bitstring.BitArray(msg)
            bits = data.bin[start:end]
            sub = bitstring.Bits(bin=bits)

            # convert to data type for the current value
            if self.datatypes[i] == "float":
                self.data[i] = sub.floatle
            elif self.datatypes[i] == "int":
                self.data[i] = sub.int
            elif self.datatypes[i] == "uint":
                self.data[i] = sub.uint
            else:
                self.data[i] = 0

    def get_value(self, index):
        """
        Returns the specified value from the most recent status message.

        @param index: index of the desired data value
        @type index: int
        @return: specified value from the most recent status message.
        """
        return self.data[index]
