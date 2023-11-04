import numpy as np

class RenderSystem:
    def __init__(self, odometer, goal_controller, gap_detector, environment):
        """
            Constructor
        """
        self._odometer = odometer
        self._goal_controller = goal_controller
        self._gap_detector = gap_detector
        self._environment = environment

        self._trajectory_x = []  # List to store x coordinates of trajectory
        self._trajectory_y = []  # List to store y coordinates of trajectory

        self.robot_width = .6   # Width of the robot
        self.robot_height = .3  # Height of the robot

    def draw(self, ax):
        """
        Main drawing function. Calls other drawing methods to render the robot's state.

        Parameters:
        ax : matplotlib axis object
            The axis to draw on.
        """

        ax.clear()  # Clear the previous frame
        ax.grid(True)  # Optional: Add grid for better visualization
        
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

    def plot_odometer(self, ax, pose):
        """
        Draw the robot based on its pose.

        Parameters:
        ax : matplotlib axis object
            The axis to draw on.
        pose : tuple
            The pose of the robot (x, y, theta).
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
        Draw the goals on the map.

        Parameters:
        ax : matplotlib axis object
            The axis to draw on.
        visited_goals : list of tuples
            Goals that have been visited.
        current_goal : tuple
            The current goal coordinates.
        other_goals : queue
            The remaining goals to visit.
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
        Plot the processed LIDAR data.
        
        :param robot_pos: Position of the robot as (x, y, theta).
        :param show: Whether to display the plot.
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
        Draw the best gap detected
        """
        gap = self._gap_detector._gap_goal
        if gap:
            ax.scatter(gap[0], gap[1], c='y', marker='o') 


    def plot_trajectory(self, ax, pose):
        """
        Draw the trajectory of the robot.

        Parameters:
        ax : matplotlib axis object
            The axis to draw on.
        pose : tuple
            The current pose of the robot.
        """
        self._trajectory_x.append(pose[0])
        self._trajectory_y.append(pose[1])
        
        # Plot the robot's trajectory
        ax.plot(self._trajectory_x, self._trajectory_y, 'g-', label='Trajectory')

    def plot_environment(self, ax, obstacles):
        for obstacle in obstacles:
            obstacle.draw(ax)