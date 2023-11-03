class EnvironmentSensingSystem:
    def __init__(self, odometer, environment_creator, environment, gap_detector):
        # Using environment_creator to add obstacl     es to the environment
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
        self._gap_detector.detect_gaps(pose, self.robot_width, self.robot_height)
        self._gap_detector.find_best_gap(pose, goal)

    def get_gap(self):
        return self._current_gap