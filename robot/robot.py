# Components
from .Components.Odometer import Odometer
from .Components.PidController import PidController
from .Components.SteeringController import SteeringController
from .Components.ProportionalController import ProportionalController
from .Components.GoalController import GoalController
from .Components.GapDetector import GapDetector

# Environment
from .Environment.Environment import Environment


# Systems
from .Systems.NavigationSystem import NavigationSystem
from .Systems.RenderSystem import RenderSystem
from .Systems.EnvironmentSensing import EnvironmentSensingSystem
from .Environment.EnvironmentCreater import EnvironmentCreator

class Robot:
    def __init__(self):
        """
        Initializes the Robot with necessary components and systems.
        """
        # Components/Controllers
        self.odometer = Odometer()
        self.pid = PidController()
        self.steering_controller = SteeringController()
        self.proportional_controller = ProportionalController()
        self.goal_controller = GoalController()
        self.gap_detector = GapDetector()

        # Environment
        self.environment = Environment()

        # Systems
        self.environment_sensing = EnvironmentSensingSystem(self.odometer, EnvironmentCreator(self.environment), 
                                                           self.environment, self.gap_detector, self.goal_controller)
        self.navigation = NavigationSystem(self.odometer, self.pid, 
                                            self.steering_controller, self.proportional_controller, self.goal_controller)
        #self.render = RenderSystem(self.odometer, self.goal_controller, self.gap_detector, self.environment)
        

    def update(self, dt):
        """
        Updates the navigation system with the given time step.
        """
        self.environment_sensing.update(self.goal_controller.get_current_goal())
        self.goal_controller.gap_goal = self.gap_detector.get_gap_goal()
        self.navigation.update(dt)
        # Update the lidar with the current robot pose and obstacles

    def draw(self, ax):
        """
            Renders simulation
        """
        self.render.draw(ax)
    
            
