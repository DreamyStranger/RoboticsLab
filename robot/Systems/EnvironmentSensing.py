import numpy as np
class EnvironmentSensingSystem:
    def __init__(self, odometer, environment_creator, environment, gap_detector, goal_controller):
        # Using environment_creator to add obstacles to the environment
        environment_creator.setup_default_environment()
        self._environment = environment
        self._robot_odometer = odometer
        self._gap_detector = gap_detector
        self.goal_controller = goal_controller
        
        self.robot_width = .2   # Width of the robot
        self.robot_height = .2  # Height of the robot
    
    def update(self, goal):
        pose = self._robot_odometer.get_pose()
        goal = self.goal_controller.get_current_goal()
        print(goal)
        if goal:
            self._gap_detector.update(pose, self.goal_controller.get_current_goal())