import auto_globals

from StateMachine import State

class Teleop(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
    
    def enter(self):
        auto_globals.drive_pub.publish(0, 0, 0, 0, 0, 0)
        auto_globals.steer_pub.publish(0, 0, 0, 0)

    def run(self):
        auto_globals.debug_pub.publish("TELEOPERATION ENABLED")

    def next(self):
        if not auto_globals.teleop:
            return self._stateMachine.idle

        return self._stateMachine.teleop