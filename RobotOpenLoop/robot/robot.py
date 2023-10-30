from .Components.Odometer import Odometer
from .Components.PidController import PidController
from .Systems.NavigationSystem import NavigationSystem
from .Components.SteeringController import SteeringController
from .Components.ProportionalController import ProportionalController
from .Components.GoalController import GoalController

class Robot:
    def __init__(self):
        """
        Initializes the Robot with necessary components and systems.
        """
        # Components
        self.odometer = Odometer()
        self.pid = PidController()
        self.steering_controller = SteeringController()
        self.proportional_controller = ProportionalController()
        self.goal_controller = GoalController()

        # Systems
        self.navigation = NavigationSystem(self.odometer, self.pid, 
                                            self.steering_controller, self.proportional_controller, self.goal_controller)

    def update(self, dt):
        """
        Updates the navigation system with the given time step.
        """
        self.navigation.update(dt)
    
    def plot(self, ax):
        """
            Plots current configuration of the robot
        """
        self.odometer.plot(ax)
            
