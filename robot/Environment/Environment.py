class Environment:
    """
    A class representing the environment of a robotic system. This class is responsible for managing 
    the obstacles present in the robot's environment.

    Attributes:
        obstacles (list): A list of obstacles present in the environment.
    """
    def __init__(self):
        """
        Initializes the Environment with an empty list of obstacles.
        """
        self.obstacles = []

    def add_obstacle(self, obstacle):
        """
        Adds an obstacle to the environment.

        Parameters:
            obstacle (object): An obstacle object to be added to the environment.
        """
        self.obstacles.append(obstacle)

    def point_in_obstacle(self, point):
        """
        Checks if a given point is within any of the obstacles in the environment.

        Parameters:
            point (tuple): The point to be checked, represented as (x, y).

        Returns:
            bool: Returns True if the point is within any obstacle, False otherwise.
        """
        for obstacle in self.obstacles:
            if obstacle.contains_point(point):
                return True
        return False
