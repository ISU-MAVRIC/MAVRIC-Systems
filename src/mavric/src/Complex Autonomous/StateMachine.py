
"""
Description:
- Template class used for running autonomnous state machines. Contains the run function needed for running and switching states
- Each state used by the state machine used the State template
How does a state machine work?
- A state machine works by having multiple states that run different code for different scenarios
- Example: THe turn towards waypoint state steers the rover in the direction of a waypoint, and then the drive towards waypoint state drives the rover until it reaches the waypoint
- Each state decides when the machine should switch to a different state, based on certian criteria defined in the state
- Example: The turn towards waypoint state will switch to the drive towards waypoint state once the rover is facing towards the waypoint
"""
class StateMachine:
    def __init__(self, initialState):
        self._prevState = initialState
        self._currentState = initialState

	#maybe add in? currently problems w/ idle publishing in enter() before main()
        #self._currentState.enter()
        #self._currentState.run()

    def run(self):
        """
        Description:
        - This function is called in a loop so it runs on repeat
        - This is the code that manages the state machine
        - It first checks if the state machine has changed states
        - If it changes states, then it runs the new states enter function
        - Finally it runs the state
        """
        self._prevState = self._currentState
        self._currentState = self._currentState.next()

        if self._prevState != self._currentState:
            self._currentState.enter()
        
        self._currentState.run()


"""
Description:
- Template used by all states. Each state overides the templates functions, this template just makes sure that all states use the correct functions
- If you are creating a new state class, make sure it has the 3 functions of the state template
"""
class State:
    def enter(self):
        """
        Description:
        - This function runs once when the state machine starts using the state
        - It is used for setting up the state or for code that needs to run in-between states
        """
        pass

    def run(self):
        """
        Description:
        - This function runs after the enter function and runs repeatedly in a loop
        - It is used executing code to control the rover
        """
        pass

    def next(self):
        """
        Description:
        - This function runs before the run function and returns a state object
        - It is used to decide if the state machine should continue to use this state or change to a different state
        - If the criteria are not met to change to a different state this function returns the current state, else it returns a different state
        """
        pass
