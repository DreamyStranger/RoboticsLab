class InputSystem:
    """
    A class responsible for handling external inputs and updating the robot's state accordingly.

    Attributes:
        _robot: A reference to the robot instance which is controlled by this input system.
    """

    def __init__(self, robot):
        """
        Initializes the InputSystem with a reference to the robot.

        Parameters:
            _state_machine (StateMachine): The Finite State Machine of the robot.
        """
        self._state_machine = robot.state_machine

    def update(self, input_data):
        """
        Receives input data and updates the robot's state based on that input.

        Parameters:
            input_data: The input received, which determines the robot's new state.
        """
        if input_data == "idle":
            self._state_machine.change_state("Idle")
        elif input_data == "heart":
            self._state_machine.change_state("Heart")
        elif input_data == "to_goal":
            self._state_machine.change_state("ToGoal")
        elif input_data == "follow":
            self._state_machine.change_state("Follow")
        else:
            print(f"Received unknown input: {input_data}")
