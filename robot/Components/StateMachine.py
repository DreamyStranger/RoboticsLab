from enum import Enum

class State(Enum):
    Idle = 1
    Heart = 2
    ToGoal = 3
    Follow = 4
    Stop = 5

class SuperState(Enum):
    Simulation = 1
    Real = 2

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
        self._current_superstate = SuperState.Simulation
        self._robot = robot
        self._goal_controller = robot.goal_controller

    def change_state(self, new_state_name, path_x, path_y):
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
                if self._current_state != State.Idle:
                    self._goal_controller.add_goals(path_x, path_y)
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
    
    def is_state(self, state_name):
        """
        Checks if the current state of the robot matches the specified state.

        Parameters:
            state_name (str): The name of the state to check. 

        Returns:
            bool: True if the current state matches the specified state, 
                  False otherwise or if the state name is not recognized.
        """
        # Check if state_name is a valid State member
        if state_name in State.__members__:
            state = State[state_name]
            # Compare the current state with the specified state
            if self._current_state == state:
                return True
        else:
            print(f"State '{state_name}' not recognized.")

        return False

    def change_superstate(self, new_superstate_name):
        """
        Changes the superstate of the robot to a new specified superstate.

        Parameters:
            new_superstate_name (str): The string name of the new superstate to transition to.
        """
        # Check if new_superstate_name is a valid SuperState member
        if new_superstate_name in SuperState.__members__:
            new_superstate = SuperState[new_superstate_name]
            # Update current state to new superstate if different
            if self._current_superstate != new_superstate:
                self._current_superstate = new_superstate
        else:
            print(f"State '{new_superstate_name}' not recognized.")

    def is_superstate(self, superstate_name):
        """
        Checks if the current superstate of the robot matches the specified superstate.

        Parameters:
            superstate_name (str): The name of the superstate to check. 

        Returns:
            bool: True if the current superstate matches the specified superstate, 
                  False otherwise or if the superstate name is not recognized.
        """
        # Check if superstate_name is a valid SuperState member
        if superstate_name in SuperState.__members__:
            superstate = SuperState[superstate_name]
            # Compare the current superstate with the specified superstate
            if self._current_superstate == superstate:
                return True
        else:
            print(f"State '{superstate_name}' not recognized.")

        return False