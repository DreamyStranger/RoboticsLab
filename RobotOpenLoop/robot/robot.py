# Components
from .Components.Odometer import Odometer
from .Components.PidController import PidController
from .Components.SteeringController import SteeringController
from .Components.ProportionalController import ProportionalController
from .Components.GoalController import GoalController
from .Components.Lidar import Lidar

# Systems
from .Systems.NavigationSystem import NavigationSystem
from .Systems.RenderSystem import RenderSystem

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

        # Systems
        self.navigation = NavigationSystem(self.odometer, self.pid, 
                                            self.steering_controller, self.proportional_controller, self.goal_controller)
        self.render = RenderSystem(self.odometer, self.goal_controller, self.lidar)

    def update(self, dt):
        """
        Updates the navigation system with the given time step.
        """
        self.navigation.update(dt)

    def draw(self, ax):
        """
            Renders simulation
        """
        self.render.draw(ax)
    
            
