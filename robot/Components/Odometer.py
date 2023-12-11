from math import cos, sin, pi

class Odometer():
    """
    A class to represent an odometer for a robotic system, tracking the robot's pose and velocities.

    Attributes:
        _pose (list): The current pose of the robot, represented as [x, y, theta].
        _vel (float): The current linear velocity of the robot.
        _steering (float): The current steering angle or angular velocity of the robot.
        max_vel (float): The maximum allowable linear velocity.
        cruise_vel (float): The cruising velocity set for the robot.
    """
    def __init__(self):
        """
        Initializes the Odometer with default values for pose, velocities, and configurable parameters.
        """
        self._pose = [0, 0, pi/2]  
        self._vel = 0  
        self._steering = 0
        self.max_vel = .6 
        self.cruise_vel = .5

    def transform(self, pose):
        """
        Updates the internal pose of the odometer by adding the input pose changes.

        Parameters:
            pose (list): The changes to be applied to the pose, represented as [delta_x, delta_y, delta_theta].
        """
        n = len(self._pose)
        for i in range(n):
            self._pose[i] += pose[i]

    def update(self, dt):
        """
        Updates the odometer's internal state based on the current linear velocity, steering angle, and time step.

        Parameters:
            dt (float): The time step for updating the odometer.
        """
        x_d = self._vel * cos(self._pose[2]) * dt  # change in x
        y_d = self._vel * sin(self._pose[2]) * dt  # change in y
        theta_d = self._steering * dt  # change in theta
        self.transform([x_d, y_d, theta_d])  # update the pose

    def reset(self):
        """
        Resets the robot's velocities (linear and angular) to zero.
        """
        self._vel = 0
        self._steering = 0

    # Getter methods to access internal state variables
    def get_pose(self):
        """
        Retrieves the current pose of the robot.

        Returns:
            list: The current pose [x, y, theta].
        """
        return self._pose
    
    def get_theta(self):
        """
        Retrieves the current orientation (theta) of the robot.

        Returns:
            float: The current orientation theta.
        """
        return self._pose[2]

    def get_vel(self):
        """
        Retrieves the current linear velocity of the robot.

        Returns:
            float: The current linear velocity.
        """
        return self._vel

    def get_velocities(self):
        """
        Retrieves the current linear and angular velocities as a tuple.

        Returns:
            tuple: The current linear and angular velocities (linear_vel, angular_vel).
        """
        linear_vel = self._vel
        angular_vel = self._steering
        return linear_vel, angular_vel

    # Setter methods to modify internal state variables
    def set_pose(self, pose):
        """
        Sets the robot's pose to the specified values.

        Parameters:
            pose (list): The new pose values [x, y, theta] to set.
        """
        self._pose = pose

    def set_vel(self, vel):
        """
        Sets the robot's linear velocity to the specified value.

        Parameters:
            vel (float): The new linear velocity to set.
        """
        self._vel = vel

    def set_theta(self, theta):
        """
        Sets the robot's orientation (theta) to the specified value.

        Parameters:
            theta (float): The new orientation theta to set.
        """
        self._pose[2] = theta

    def set_steering(self, steering):
        """
        Sets the robot's steering angle to the specified value.

        Parameters:
            steering (float): The new steering angle to set.
        """
        self._steering = steering