import numpy as np
class EnvironmentSensingSystem:
    def __init__(self, odometer, environment_creator, environment, gap_detector):
        # Using environment_creator to add obstacles to the environment
        environment_creator.setup_default_environment()
        self._environment = environment
        self._robot_odometer = odometer
        self._gap_detector = gap_detector
        
        self.robot_width = .6   # Width of the robot
        self.robot_height = .3  # Height of the robot
    
    def update(self, goal):
        if not goal:
            return
        pose = self._robot_odometer.get_pose()
        self._gap_detector.find_gaps()
        self._gap_detector.process_gaps(self.robot_width)
        goal_angle = self.calculate_goal_angle(pose, goal)

        self._gap_detector.determine_best_gap(pose[2], goal_angle)

        

    def get_gap(self):
        return self._current_gap
    
    
    def calculate_goal_angle(self, robot_pose, goal_pos):
        """
        Calculates the bearing or angle to the goal in radians from the robot's current position.
        
        :param robot_pos: Tuple containing the robot's current position (x, y).
        :param goal_pos: Tuple containing the goal's position (x, y).
        :return: Angle to the goal in radians.
        """
        dx = goal_pos[0] - robot_pose[0]
        dy = goal_pos[1] - robot_pose[1]
        return np.arctan2(dy, dx)