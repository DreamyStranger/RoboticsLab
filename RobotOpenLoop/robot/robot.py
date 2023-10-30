# Ours
from .Components.Odometer import Odometer
from .Components.PidController import PidController
from .Systems.NavigationSystem import NavigationSystem
from .Components.SteeringController import SteeringController
from .Components.ProportionalController import ProportionalController
from .Components.GoalController import GoalController
# Python
import matplotlib.pyplot as plt
import numpy as np

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
        Plots the robot's current configuration
        """
        width = .1  # Width of the robot representation
        height = .05  # Height of the robot representation
        pose = self.odometer.get_pose()
        
        # Rectangle vertices in local(robot's) coordinate frame
        
        corners = np.array([
            [-width/2, -height/2],  # Bottom-left corner
            [-width/2, height/2],   # Top-left corner
            [width/2, height/2],    # Top-right corner
            [width/2, -height/2]    # Bottom-right corner
        ])
        
        # Rotation Matrix
        rotmat = np.array([
            [np.cos(pose[2]), -np.sin(pose[2])],
            [np.sin(pose[2]), np.cos(pose[2])]
        ])
        
        # Transformation
        corners_new = np.dot(rotmat, corners.T).T + pose[:2]
        
        # Create a closed rectangle for plotting
        robot = np.vstack([corners_new, corners_new[0, :]])
        
        # Plot the robot's shape
        ax.plot(robot[:, 0], robot[:, 1], 'k-')
            
