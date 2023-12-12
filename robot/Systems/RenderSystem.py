import numpy as np
import matplotlib.pyplot as plt

class RenderSystem:
    """
    A class responsible for rendering and visualizing the state of a robot and its environment. It provides 
    functionalities to draw the robot, its trajectory, goals, and detected gaps, as well as the environment obstacles.

    Attributes:
        _odometer (Odometer): The odometer component of the robot for obtaining its pose.
        _goal_controller (GoalController): The goal controller of the robot for accessing current and visited goals.
        _gap_detector (GapDetector): The gap detector component of the robot for gap visualization.
        _environment (Environment): The environment in which the robot operates.
        _state_machine (StateMachine): The Finite State Machine of the robot.
        _trajectory_x (list): List to store the x coordinates of the robot's trajectory.
        _trajectory_y (list): List to store the y coordinates of the robot's trajectory.
        robot_width (float): The width of the robot, used in visualization.
        robot_height (float): The height of the robot, used in visualization.
    """

    def __init__(self, robot):
        """
        Initializes the RenderSystem with necessary components from the robot.

        Parameters:
            robot (Robot): The robot instance containing necessary components for rendering.
        """
        self._odometer = robot.odometer
        self._goal_controller = robot.goal_controller
        self._gap_detector = robot.gap_detector
        self._environment = robot.environment
        self._state_machine = robot.state_machine

        self._trajectory_x = []  
        self._trajectory_y = []
        self.robot_width = .6   
        self.robot_height = .3

    def draw(self, ax):
        """
        Main drawing function that calls other methods to render the robot's state and environment.

        Parameters:
            ax (matplotlib.axes.Axes): The matplotlib axis object to draw on.
        """
        if self._state_machine.is_superstate("Real"):
            return

        ax.clear()  # Clear the previous frame
        ax.grid(True)  # Add grid for better visualization
        
        # Retrieve necessary states
        pose = self._odometer.get_pose()
        visited_goals = self._goal_controller.get_visited()
        current_goal = self._goal_controller.get_current_goal()
        other_goals = self._goal_controller.get_goals()
        obstacles = self._environment.obstacles
        lidar_data = self._gap_detector.processed_lidar_data

        # Draw
        self.plot_lidar(ax, lidar_data, pose)
        self.plot_odometer(ax, pose)
        self.plot_goals(ax, visited_goals, current_goal, other_goals)
        self.plot_gap(ax)
        self.plot_trajectory(ax, pose)
        self.plot_environment(ax, obstacles)

        # Limit workspace 
        ax.set_xlim([-5, 5])
        ax.set_ylim([-5, 5])

        plt.pause(0.1)  # Pause to update the display

    def plot_odometer(self, ax, pose):
        """
        Draws the robot on the given matplotlib axis based on its current pose.

        Parameters:
            ax (matplotlib.axes.Axes): The axis to draw the robot on.
            pose (tuple): The pose of the robot (x, y, theta).
        """
        width = self.robot_width
        height = self.robot_height

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

        # Plot
        ax.plot(robot[:, 0], robot[:, 1], 'k-')


    def plot_goals(self, ax, visited_goals, current_goal, other_goals):
        """
        Draws the goals (visited, current, and remaining) on the map.

        Parameters:
            ax (matplotlib.axes.Axes): The axis to draw the goals on.
            visited_goals (list): List of visited goals.
            current_goal (tuple): The current goal coordinates.
            other_goals (queue): Queue of remaining goals.
        """
        # Plot visited goals
        for point in visited_goals:
            ax.scatter(point[0], point[1], c='g', marker='o') 

        # Plot the rest of the goals in the queue
        for point in list(other_goals.queue):
            ax.scatter(point[0], point[1], c='r', marker='o') 
        
        # Plot current goal
        if current_goal:
            ax.scatter(current_goal[0], current_goal[1], c='b', marker='o')

    def plot_lidar(self, ax, lidar_data, robot_pose):
        """
        Plots the processed LIDAR data on the given axis, showing the detected distances.

        Parameters:
            ax (matplotlib.axes.Axes): The axis to plot the LIDAR data on.
            lidar_data (list): The processed LIDAR data.
            robot_pose (tuple): The current pose of the robot.
        """
        # Define angles for processed LIDAR data (-90 to +90 degrees)
        total_segments = len(lidar_data)
        angles = np.linspace(-np.pi/2, np.pi/2, total_segments)

        # Plot processed LIDAR data
        for angle, distance in zip(angles, lidar_data):
            adjusted_angle = angle + robot_pose[2]  # Adjust the angle based on the robot's orientation
            xr = robot_pose[0] + distance * np.cos(adjusted_angle)
            yr = robot_pose[1] + distance * np.sin(adjusted_angle)
            ax.plot([robot_pose[0], xr], [robot_pose[1], yr], 'y--')

    def plot_gap(self, ax):
        """
        Draws the best gap detected by the gap detector.

        Parameters:
            ax (matplotlib.axes.Axes): The axis to draw the gap on.
        """
        gap = self._gap_detector._gap_goal
        if gap:
            ax.scatter(gap[0], gap[1], c='y', marker='o') 


    def plot_trajectory(self, ax, pose):
        """
        Draws the trajectory of the robot based on its movement history.

        Parameters:
            ax (matplotlib.axes.Axes): The axis to draw the trajectory on.
            pose (tuple): The current pose of the robot.
        """
        self._trajectory_x.append(pose[0])
        self._trajectory_y.append(pose[1])
        
        # Plot the robot's trajectory
        ax.plot(self._trajectory_x, self._trajectory_y, 'g-', label='Trajectory')

    def plot_environment(self, ax, obstacles):
        """
        Draws the obstacles from the environment on the given axis.

        Parameters:
            ax (matplotlib.axes.Axes): The axis to draw the environment on.
            obstacles (list): List of obstacles in the environment.
        """
        for obstacle in obstacles:
            obstacle.draw(ax)