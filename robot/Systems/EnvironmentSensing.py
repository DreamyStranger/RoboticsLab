import numpy as np
class EnvironmentSensingSystem:
    """
    A class that represents the environment sensing system of a robot. It integrates various components 
    to perceive and interact with the environment, particularly focusing on obstacle detection and goal-oriented navigation.

    Attributes:
        _environment (Environment): The environment in which the robot operates.
        _robot_odometer (Odometer): The odometer component of the robot for tracking its pose.
        _gap_detector (GapDetector): The gap detection component for identifying navigable spaces.
        goal_controller (GoalController): The goal management component of the robot.
        _state_machine (StateMachine): The Finite State Machine of the robot.
        robot_width (float): The width of the robot, used for sensing calculations.
        robot_height (float): The height of the robot, used for sensing calculations.
    """

    def __init__(self, robot, environment_creator):
        """
        Initializes the EnvironmentSensingSystem with references to the robot's components and the environment creator.

        Parameters:
            robot (Robot): The robot instance containing necessary components like the odometer and gap detector.
            environment_creator (EnvironmentCreator): The environment creator to set up the default environment.
        """
        environment_creator.setup_default_environment()
        self._environment = robot.environment
        self._robot_odometer = robot.odometer
        self._gap_detector = robot.gap_detector
        self.goal_controller = robot.goal_controller
        self._state_machine = robot.state_machine
        
        self.robot_width = .2   # Width of the robot
        self.robot_height = .2  # Height of the robot
    
    def update(self, goal):
        """
        Updates the environment sensing system based on the robot's current pose and goal. 
        This method primarily uses the gap detector to assess navigable paths in the environment.

        Parameters:
            goal (tuple): The current goal coordinates (x, y) that the robot is navigating towards.
        """
        if self._state_machine.is_superstate("Real"):
            return
        pose = self._robot_odometer.get_pose()
        goal = self.goal_controller.get_current_goal()
        if goal:
            self._gap_detector.update(pose)