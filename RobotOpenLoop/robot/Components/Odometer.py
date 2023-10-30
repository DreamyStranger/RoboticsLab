from math import pi, cos, sin, atan2
import matplotlib.pyplot as plt
import numpy as np
pi
class Odometer():
    def __init__(self):
        """
        Constructor: Initializes the odometer component with default values.
        """
        # private variables representing the internal state
        self._pose = [0, 0, 0]  # pose of the robot [x, y, theta]
        self._vel = 0  # linear velocity
        self._steering = 0  # steering angle or angular velocity
        
        # public variables representing the configurable parameters
        self.max_vel = 1  # maximum allowable velocity
        self.cruise_vel = 0.5  # desired cruising velocity

    def transform(self, pose):
        """
        Updates the internal pose of the odometer by adding the input pose.
        """
        n = len(self._pose)
        for i in range(n):
            self._pose[i] += pose[i]

    def update(self, dt):
        """
        Updates the internal state of the odometer based on the current velocity,
        steering angle, and the time step.
        """
        x_d = self._vel * cos(self._pose[2]) * dt  # change in x
        y_d = self._vel * sin(self._pose[2]) * dt  # change in y
        theta_d = self._steering * dt  # change in theta
        self.transform([x_d, y_d, theta_d])  # update the pose

    def reset(self):
        """
        Resets the velocities to zero.
        """
        self._vel = 0
        self._steering = 0
    
    def plot(self, ax):
        """
        Plots the robot's current configuration
        """
        width = .1  # Width of the robot representation
        height = .05  # Height of the robot representation
        
        # Rectangle vertices in local(robot's) coordinate frame
        corners = np.array([
            [-width/2, -height/2],  # Bottom-left corner
            [-width/2, height/2],   # Top-left corner
            [width/2, height/2],    # Top-right corner
            [width/2, -height/2]    # Bottom-right corner
        ])
        
        # Rotation Matrix
        rotmat = np.array([
            [np.cos(self._pose[2]), -np.sin(self._pose[2])],
            [np.sin(self._pose[2]), np.cos(self._pose[2])]
        ])
        
        # Transformation
        corners_new = np.dot(rotmat, corners.T).T + self._pose[:2]
        
        # Create a closed rectangle for plotting
        robot = np.vstack([corners_new, corners_new[0, :]])
        
        # Plot the robot's shape
        ax.plot(robot[:, 0], robot[:, 1], 'k-')


    # Getter methods to access internal state variables
    def get_pose(self):
        return self._pose
    
    def get_theta(self):
        return self._pose[2]

    def get_vel(self):
        return self._vel

    def get_velocities(self):
        """
        Returns the linear and angular velocities as a tuple.
        """
        linear_vel = self._vel
        angular_vel = self._steering
        return linear_vel, angular_vel

    # Setter methods to modify internal state variables
    def set_pose(self, pose):
        self._pose = pose

    def set_vel(self, vel):
        self._vel = vel

    def set_theta(self, theta):
        self._pose[2] = theta

    def set_steering(self, steering):
        self._steering = steering