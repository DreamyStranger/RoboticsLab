from enum import Enum

class State(Enum):
    Idle = 1
    Heart = 2
    ToGoal = 3
    Follow = 4

class StateMachine:
    """
    A class that manages the states of a robot using a finite state machine approach.

    Attributes:
        _current_state (State): The current state of the robot from the State enumeration.
        _robot: The robot instance that this state machine is controlling.

    Methods:
        change_state: Changes the robot's state using the string name of the target state.
        enter_state: Handles the actions to be performed when entering a new state.
        get_state: Returns the current state of the robot.
    """

    def __init__(self, robot):
        """
        Initializes the StateMachine with the robot instance and sets the initial state to State.Idle.

        Parameters:
            robot: The robot instance that this state machine will control.
        """
        self._current_state = State.Idle
        self._robot = robot

    def change_state(self, new_state_name):
        """
        Changes the state of the robot based on the string name of the new state. It calls the enter_state 
        method to handle the transition.

        Parameters:
            new_state_name (str): The string name of the new state to transition to.
        """
        #print("Received state name:", new_state_name)  # Debugging 
        #print("Available states:", State.__members__)  # Debugging 

        if new_state_name in State.__members__:
            new_state = State[new_state_name]
            if self._current_state != new_state:
                self._current_state = new_state
                self.enter_state(new_state)
        else:
            print(f"State '{new_state_name}' not recognized.")

    def enter_state(self, state):
        """
        Handles entering a new state. This method resets the robot and prints the current state value.
        It can be extended to include state-specific initialization or actions.

        Parameters:
            state (State): The new state the robot is entering.
        """
        self._current_state = state
        self._robot.reset()
        print(f"Entered state: {state.name} (value: {state.value})")

    def get_state(self):
        """
        Returns the current state of the robot.

        Returns:
            State: The current state from the State enumeration.
        """
        return self._current_state
    
    def is_state(self, state_name):
        if state_name in State.__members__:
            state = State[state_name]
            if self._current_state == state:
                return True
        else:
            print(f"State '{state_name}' not recognized.")

        return False