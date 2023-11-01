import matplotlib.pyplot as plt
import numpy as np

class RenderSystem:
    def __init__(self, odometer, goal_controller, lidar):
        self._odometer  = odometer
        self._goal_controller = goal_controller
        self._lidar = lidar

        self._trajectory_x = []
        self._trajectory_y = []
         
        self.robot_width = .6
        self.robot_height = .3

    def draw(self, ax):
        # Stats
        pose = self._odometer.get_pose()
        visited_goals =  self._goal_controller.get_visited()
        current_goal = self._goal_controller.get_current_goal()
        other_goals = self._goal_controller.get_goals()
        
        # Plot
        self.plot_lidar(ax, pose, [])
        self.plot_goals(ax, visited_goals, current_goal, other_goals)
        self.plot_odometer(ax, pose)
        self.plot_trajectory(ax, pose)
        ax.set_xlim([-5, 5])
        ax.set_ylim([-5, 5])

    def plot_odometer(self, ax, pose):
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
        # Plot visited goals
        for point in visited_goals:
            ax.scatter(point[0], point[1], c='g', marker='o') 

        # Plot the rest of the goals in the queue
        for point in list(other_goals.queue):
            ax.scatter(point[0], point[1], c='r', marker='o') 
        
        # Plot current goal
        if current_goal:
            ax.scatter(current_goal[0], current_goal[1], c='b', marker='o')

    def plot_lidar(self, ax, pose, obstacles):
        self._lidar.plot(ax, pose, obstacles)

    def plot_trajectory(self, ax, pose):
        self._trajectory_x.append(pose[0])
        self._trajectory_y.append(pose[1])
        
        # Plot the robot's trajectory
        ax.plot(self._trajectory_x, self._trajectory_y, 'g-', label='Trajectory')