from .Components.OdometerComponent import OdometerComponent
from .Components.PidComponent import PidComponent
from .Systems.NavigationSystem import NavigationSystem
from .Components.SteeringController import SteeringController
from .Components.ProportionalController import ProportionalController

class Robot:
    def __init__(self):
        """
        Initializes the Robot with necessary components and systems.
        """
        # Components
        self.odometer = OdometerComponent()
        self.pid = PidComponent()
        self.steering_controller = SteeringController()
        self.proportional_controller = ProportionalController()

        # Systems
        self.navigation = NavigationSystem(self.odometer, self.pid, 
                                            self.steering_controller, self.proportional_controller)

    def update(self, dt):
        """
        Updates the navigation system with the given time step.
        """
        self.navigation.update(dt)

    def set_max_steering(self, steering):
        """
        Sets the maximum steering angle of the odometer.
        """
        self.odometer.max_steering = steering

    def set_cruise_vel(self, vel):
        """
        Sets the cruise velocity of the odometer.
        """
        self.odometer.cruise_vel = vel
    
    def plot(self, ax):
        """
            Plots current configuration of the robot
        """
        self.odometer.plot(ax)
            
