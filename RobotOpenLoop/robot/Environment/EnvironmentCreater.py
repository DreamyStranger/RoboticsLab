from .RectangleObstacle import RectangleObstacle

class EnvironmentCreator:
    def __init__(self, environment):
        self.environment = environment

    def add_rectangle_obstacle(self, x, y, width, height):
        rectangle = RectangleObstacle(x, y, width, height)
        self.environment.add_obstacle(rectangle)
        return rectangle

    def setup_default_environment(self):
        # Add a set of predefined obstacles to the environment
        self.add_rectangle_obstacle(- 5, -2, 1, 10)
        self.add_rectangle_obstacle(- 5, 0, 4, 1)
        self.add_rectangle_obstacle(1, 0, 4, 1)
        self.add_rectangle_obstacle(-6, -2, 2, 4)
        self.add_rectangle_obstacle(4, -2, 2, 4)

        # Add more obstacles as needed