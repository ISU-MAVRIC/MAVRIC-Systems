class StateMachine:
    def __init__(self, initialState):
        self._prevState = initialState
        self._currentState = initialState

        self._currentState.enter()
        self._currentState.run()

    def run(self):
        self._prevState = self._currentState
        self._currentState = self._currentState.next()

        if self._prevState != self._currentState:
            self._currentState.enter()
        
        self._currentState.run()

class State:
    def enter(self):
        pass

    def run(self):
        pass

    def next(self):
        pass
