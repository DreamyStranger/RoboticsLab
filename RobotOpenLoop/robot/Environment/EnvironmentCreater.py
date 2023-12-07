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
        self.add_rectangle_obstacle(- 5, 1, 4.5, 10)
        self.add_rectangle_obstacle(- 5, 0, 4, 1)
        self.add_rectangle_obstacle(1, 0, 4, 1)
        self.add_rectangle_obstacle(-6, -2, 2, 4)
        self.add_rectangle_obstacle(4, 0, 2, 10)
        self.add_rectangle_obstacle(2, 0, 10, 4)
        self.add_rectangle_obstacle(1, 4, 2, 2)
        self.add_rectangle_obstacle(0, 6, 1, 5)

        # Add more obstacles as needed