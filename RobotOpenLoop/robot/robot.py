# Components
from .Components.Odometer import Odometer
from .Components.PidController import PidController
from .Components.SteeringController import SteeringController
from .Components.ProportionalController import ProportionalController
from .Components.GoalController import GoalController
from .Components.Lidar import Lidar
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
        self.lidar = Lidar()

        # Environment
        self.environment = Environment()

        # Systems
        self.navigation = NavigationSystem(self.odometer, self.pid, 
                                            self.steering_controller, self.proportional_controller, self.goal_controller)
        self.render = RenderSystem(self.odometer, self.goal_controller, self.lidar, self.environment)
        self.environment_sensing = EnvironmentSensingSystem(self.odometer, EnvironmentCreator(self.environment), self.environment, self.lidar)

    def update(self, dt):
        """
        Updates the navigation system with the given time step.
        """
        
        self.navigation.update(dt)
        # Update the lidar with the current robot pose and obstacles
        self.lidar.update(self.odometer.get_pose(), self.environment.obstacles)

    def draw(self, ax):
        """
            Renders simulation
        """
        self.render.draw(ax)
    
            
