from abc import ABC, abstractmethod
from std_msgs.msg import Bool
import rospy

class LED(ABC):

    def __init__(self):
        self.toggled = False

    def toggle(self):
        self.toggled = not self.toggled
    
    def isToggled(self):
        return self.toggled

    @abstractmethod
    def setColor(self):
        pass

    @abstractmethod
    def setBrightness(self):
        pass

class LightPole(LED):

    def __init__(self):
        super().__init__()
        self.prevColorPub = None
        self.redPub = rospy.Publisher('/relay/light_pole/red', Bool, queue_size=10)
        self.bluePub = rospy.Publisher('/relay/light_pole/blue', Bool, queue_size=10)
        self.greenPub = rospy.Publisher('/relay/light_pole/green', Bool, queue_size=10)

    def toggle(self):
        super().toggle()
        self.prevColorPub.publish(self.toggled)

    def setColor(self, color):
        if self.prevColorPub is not None:   
            self.prevColorPub.publish(False)
        if color == "RED":
            self.redPub.publish(True)
            self.prevColorPub = self.redPub
        elif color == "BLUE":
            self.bluePub.publish(True)
            self.prevColorPub = self.bluePub
        elif color == "GREEN":
            self.greenPub.publish(True)
            self.prevColorPub = self.greenPub
        else:
            self.redPub.publish(False)
            self.bluePub.publish(False)
            self.greenPub.publish(False)


    def setBrightness(self, brightness):
        pass


class LightStrip(LED):
    pass