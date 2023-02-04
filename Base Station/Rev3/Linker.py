import pygame
from abc import ABC

"""
Objects for linking UI widgets to program features (widget to pygame input, display to value)
"""


class Controllable(ABC):
    def __int__(self, widget, controlmap, valuemap):
        self.widget = widget
        self.controlMap = controlmap
        self.valueMap = valuemap
        self.

    def link(self):
        """
        Function for linking controller inputs to widgets
        """

        data = self.widget.property("Controls")
        countnum = 1
        for setting in data:
            if "controller" in setting:
                countnum = setting.split(" ")[1]
            elif ""