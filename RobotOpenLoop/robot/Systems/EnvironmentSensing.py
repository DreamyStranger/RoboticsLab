class EnvironmentSensingSystem:
    def __init__(self, odometer, environment_creator, environment, lidar):
        # Using environment_creator to add obstacles to the environment
        environment_creator.setup_default_environment()
        self._environment = environment
        self._lidar = lidar
        self._robot_odometer = odometer
    
    def update(self): 
        self.lidar.update(self._robot_odometer.get_pose(), self.environment.obstacles)
