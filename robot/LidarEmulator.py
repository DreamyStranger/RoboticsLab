import numpy as np

class LidarEmulator:
    """
    A class to emulate LIDAR sensor data for a robotic system. This emulator simulates the behavior of a LIDAR sensor 
    by casting rays in the environment and detecting obstacles.

    Attributes:
        _num_rays (int): The number of rays used in the simulation to represent LIDAR data.
        max_distance (float): The maximum distance that the LIDAR can detect.
        lidar_data (list): The simulated distances detected by each LIDAR ray.
        lidar_end_points (list): The end points of each LIDAR ray in the environment.
    """
    def __init__(self, num_rays=720, max_distance=5):
        """
        Initializes the LidarEmulator with a specified number of rays and maximum detection distance.

        Parameters:
            num_rays (int): The number of rays to use in the LIDAR simulation.
            max_distance (float): The maximum distance that the LIDAR rays can detect.
        """
        self._num_rays = num_rays
        self.max_distance = max_distance
        self.lidar_data = [max_distance] * num_rays
        self.lidar_end_points = [(0, 0)] * num_rays

    def cast_ray(self, x, y, angle, obstacles):
        """
        Casts a single ray in the environment and checks for collisions with obstacles.

        Parameters:
            x (float): The x-coordinate of the ray's starting point.
            y (float): The y-coordinate of the ray's starting point.
            angle (float): The angle at which the ray is cast.
            obstacles (list): A list of obstacles to check for collisions.

        Returns:
            float: The distance at which an obstacle is detected, or the maximum distance if no obstacle is found.
        """
        for r in np.linspace(0, self.max_distance, 200):
            xr = x + r * np.cos(angle)
            yr = y + r * np.sin(angle)

            for obstacle in obstacles:
                if obstacle.contains_point((xr, yr)):
                    return r   

        return self.max_distance
    
    def update(self, robot_pos, obstacles):
        """
        Updates the LIDAR emulation data based on the robot's position and the environment's obstacles.

        Parameters:
            robot_pos (tuple): The position of the robot, given as (x, y, theta).
            obstacles (list): A list of obstacles in the environment to check against the LIDAR rays.
        """
        angles = np.linspace(0, 2 * np.pi, self._num_rays, endpoint=False)
        for i, angle in enumerate(angles):
            adjusted_angle = angle + robot_pos[2]  # Adjust the angle based on the robot's orientation
            distance = self.cast_ray(robot_pos[0], robot_pos[1], adjusted_angle, obstacles)
            self.lidar_data[i] = distance

            xr = robot_pos[0] + distance * np.cos(adjusted_angle)
            yr = robot_pos[1] + distance * np.sin(adjusted_angle)
            self.lidar_end_points[i] = (xr, yr)

    def get_end_points(self):
        """
        Retrieves the end points of all the LIDAR rays in the environment.

        Returns:
            list: A list of tuples representing the end points of the LIDAR rays.
        """
        return self.lidar_end_points
    
    def get_data(self):
        """
        Retrieves the simulated LIDAR data.

        Returns:
            list: The distances detected by each LIDAR ray.
        """
        return self.lidar_data

    def get_num_rays(self):
        """
        Retrieves the number of rays used in the LIDAR simulation.

        Returns:
            int: The number of rays.
        """
        return self._num_rays
