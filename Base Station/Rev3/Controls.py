

"""
Set of object pertaining to assigning and manipulating inputs
"""


class Controls:
    def __init__(self, controllers):
        """
        Object for storing controllable widgets for each controller input
        """

        self.controllers = controllers
        self.controlIndex = [{}]*len(controllers)
        for controller in self.controllers:
