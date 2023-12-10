# Components
from .Components.Odometer import Odometer
from .Components.PidController import PidController
from .Components.SteeringController import SteeringController
from .Components.ProportionalController import ProportionalController
from .Components.GoalController import GoalController
from .Components.GapDetector import GapDetector
from .Components.StateMachine import StateMachine

# Environment
from .Environment.Environment import Environment
from .Environment.EnvironmentCreater import EnvironmentCreator

# Systems
from .Systems.NavigationSystem import NavigationSystem
from .Systems.RenderSystem import RenderSystem
from .Systems.EnvironmentSensing import EnvironmentSensingSystem
from .Systems.InputSystem import InputSystem


class Robot:
    """
    A class representing a complete robot system. It integrates various components and systems necessary for 
    the robot's operation, including navigation, environment sensing, and rendering.

    Attributes:
        odometer (Odometer): The odometer component for tracking the robot's pose.
        pid (PidController): The PID controller for velocity control.
        steering_controller (SteeringController): The steering controller for navigation.
        proportional_controller (ProportionalController): An alternative controller for proportional control.
        goal_controller (GoalController): The controller for managing navigation goals.
        gap_detector (GapDetector): The component for detecting gaps in the environment.
        state_machine (StateMachine): The Finite State Machine of the robot.
        environment (Environment): The representation of the robot's operating environment.
        environment_sensing (EnvironmentSensingSystem): The system for sensing and interacting with the environment.
        navigation (NavigationSystem): The system for managing the robot's navigation.
        render (RenderSystem): The system for rendering and visualizing the robot's state and environment.
        input_system (InputSystem): The system for handling external inputs and updating the robot's state accordingly.
    """
    def __init__(self):
        """
        Initializes the Robot with all necessary components and systems, setting up its functionality for navigation, 
        environment interaction, and visualization.
        """
        # Components/Controllers
        self.odometer = Odometer()
        self.pid = PidController()
        self.steering_controller = SteeringController()
        self.proportional_controller = ProportionalController()
        self.goal_controller = GoalController()
        self.gap_detector = GapDetector()
        self.state_machine = StateMachine(self)

        # Environment
        self.environment = Environment()

        # Systems
        self.environment_sensing = EnvironmentSensingSystem(self, EnvironmentCreator(self.environment))
        self.navigation = NavigationSystem(self)
        self.render = RenderSystem(self)
        self.input_system = InputSystem(self)
        

    def update(self, dt):
        """
        Updates the robot's systems based on the given time step. This includes updating navigation and environment sensing.

        Parameters:
            dt (float): The time step for updating the robot's systems.
        """
        self.input_system.update("idle")
        self.environment_sensing.update(self.goal_controller.get_current_goal())
        #self.goal_controller.gap_goal = self.gap_detector.get_gap_goal()
        self.navigation.update(dt)

    def draw(self, ax):
        """
        Renders the simulation of the robot's current state and environment on a given axis.

        Parameters:
            ax (matplotlib.axes.Axes): The matplotlib axis object on which the robot and its environment will be drawn.
        """
        self.render.draw(ax)

    def reset(self):
        """
        Resets the key components of the robot. This includes resetting the goal controller, odometer, PID controller, 
        and proportional controller.

        This method is useful for reinitializing the robot's state either for a new operation or after an interruption, 
        ensuring that it returns to a predefined initial state.
        """
        self.goal_controller.reset()
        self.odometer.reset()
        self.pid.reset()
        self.proportional_controller.reset()
    
            
