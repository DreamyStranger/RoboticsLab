from .RectangleObstacle import RectangleObstacle

class EnvironmentCreator:
    """
    A class responsible for creating and modifying the environment in which a robotic system operates. 
    It allows for the addition of various obstacles to simulate different environmental scenarios.

    Attributes:
        environment (Environment): The environment object to which obstacles are added.
    """
    def __init__(self, environment):
        """
        Initializes the EnvironmentCreator with a reference to an Environment object.

        Parameters:
            environment (Environment): The environment object that will be modified by this class.
        """
        self.environment = environment

    def add_rectangle_obstacle(self, x, y, width, height):
        """
        Adds a rectangular obstacle to the environment.

        Parameters:
            x (float): The x-coordinate of the bottom-left corner of the rectangle.
            y (float): The y-coordinate of the bottom-left corner of the rectangle.
            width (float): The width of the rectangle.
            height (float): The height of the rectangle.

        Returns:
            RectangleObstacle: The created rectangle obstacle object.
        """
        rectangle = RectangleObstacle(x, y, width, height)
        self.environment.add_obstacle(rectangle)
        return rectangle

    def setup_default_environment(self):
        """
        Sets up a default environment with a predefined set of rectangular obstacles.

        This method can be used to quickly initialize a standard testing environment for the robotic system.
        """
        #self.add_rectangle_obstacle(- 5, 1, 4.5, 10)
        #self.add_rectangle_obstacle(- 5, 0, 4, 1)
        #self.add_rectangle_obstacle(1, 0, 4, 1)
        #self.add_rectangle_obstacle(-6, -2, 2, 4)
        #self.add_rectangle_obstacle(4, 0, 2, 10)
        #self.add_rectangle_obstacle(2, 0, 10, 4)
        #self.add_rectangle_obstacle(1, 4, 2, 2)
        #self.add_rectangle_obstacle(0, 6, 1, 5)
        pass
        # Add more obstacles as needed