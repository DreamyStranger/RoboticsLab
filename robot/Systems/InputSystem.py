import numpy as np

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
        self._odometer = robot.odometer

    def update(self, input_data):
        """
        Receives input data and updates the robot's state based on that input.

        Parameters:
            input_data: The input received, which determines the robot's new state.
        """
        path_x = []
        path_y = []
        pose = self._odometer.get_pose()
        if input_data == "idle":
            self._state_machine.change_state("Idle", path_x, path_y)
        elif input_data == "heart":
            path_x, path_y = self.circle_data(pose[0], pose[1])
            self._state_machine.change_state("Heart")
        elif input_data == "to_goal":
            path_x, path_y = self.point_data(pose[0], pose[1])
            path_x.append(pose[0])
            path_y.append(pose[1])
            self._state_machine.change_state("ToGoal", path_x, path_y)
        elif input_data == "follow":
            self._state_machine.change_state("Follow", path_x, path_y)
        elif input_data == "stop":
            self._state_machine.change_state("Stop", [], [])
        else:
            print(f"Received unknown input: {input_data}")


    def circle_data(self, x, y):
        """
        Generate circular path data.

        Args:
            x (float): X-coordinate of the circle center.
            y (float): Y-coordinate of the circle center.

        Returns:
            tuple: Lists of x and y coordinates forming a circular path.
        """
        r = .5
        theta = np.linspace(0, 2*np.pi, 100)
        x_out = x + r * np.cos(theta)
        y_out = y + r * np.sin(theta)
        return x_out.tolist(), y_out.tolist()

    def point_data(self, x, y):
        """
        Generate a point path data.

        Args:
            x (float): X-coordinate of the point.
            y (float): Y-coordinate of the point.

        Returns:
            tuple: Lists of x and y coordinates forming a single point.
        """
        if self._state_machine.is_superstate("Simulation"):
            return [0], [5]
        return [x + 1], [y]