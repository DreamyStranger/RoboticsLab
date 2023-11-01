import matplotlib.pyplot as plt
import numpy as np

class Lidar:
    def __init__(self, num_rays=36, max_distance=10):
        self.num_rays = num_rays
        self.max_distance = max_distance
        self.lidar_data = [max_distance] * num_rays

    def cast_ray(self, x, y, angle, obstacles):
        for r in np.linspace(0, self.max_distance, 1000):
            xr = x + r * np.cos(angle)
            yr = y + r * np.sin(angle)

            for obstacle in obstacles:
                if obstacle.contains_point((xr, yr)):
                    return r

        return self.max_distance

    def plot(self, ax, robot_pos, obstacles):
        angles = np.linspace(0, 2 * np.pi, self.num_rays, endpoint=False)
        for i, angle in enumerate(angles):
            adjusted_angle = angle + robot_pos[2]  # Adjust the angle based on the robot's orientation
            distance = self.cast_ray(robot_pos[0], robot_pos[1], adjusted_angle, obstacles)
            self.lidar_data[i] = distance

            xr = robot_pos[0] + distance * np.cos(adjusted_angle)
            yr = robot_pos[1] + distance * np.sin(adjusted_angle)
            ax.plot([robot_pos[0], xr], [robot_pos[1], yr], 'y-')

        ax.plot(robot_pos[0], robot_pos[1], 'bo')  # Robot position
        ax.axis('equal')
