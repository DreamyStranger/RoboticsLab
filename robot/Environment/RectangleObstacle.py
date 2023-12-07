import matplotlib.patches as patches

class RectangleObstacle:
    """
    A class representing a rectangular obstacle in a robotic environment. It provides functionalities to check if a point 
    is within the obstacle and to draw the obstacle on a visualization canvas.

    Attributes:
        x (float): The x-coordinate of the bottom-left corner of the rectangle.
        y (float): The y-coordinate of the bottom-left corner of the rectangle.
        width (float): The width of the rectangle.
        height (float): The height of the rectangle.
    """
    def __init__(self, x, y, width, height):
        """
        Initializes a RectangleObstacle with the specified dimensions and location.

        Parameters:
            x (float): The x-coordinate of the bottom-left corner of the rectangle.
            y (float): The y-coordinate of the bottom-left corner of the rectangle.
            width (float): The width of the rectangle.
            height (float): The height of the rectangle.
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def contains_point(self, point):
        """
        Checks if a given point is inside the rectangular obstacle.

        Parameters:
            point (tuple): A point represented as (x, y) to be checked.

        Returns:
            bool: Returns True if the point is inside the rectangle, False otherwise.
        """
        x, y = point
        return self.x < x < self.x + self.width and self.y < y < self.y + self.height
    
    def draw(self, ax):
        """
        Draws the rectangle on a given matplotlib axis.

        Parameters:
            ax (matplotlib.axes.Axes): The matplotlib axis on which the rectangle will be drawn.
        """
        # Defining vertices of the polygon (rectangle)
        vertices = [(self.x, self.y), (self.x + self.width, self.y), 
                    (self.x + self.width, self.y + self.height), (self.x, self.y + self.height)]
        
        # Creating a Polygon patch
        polygon = patches.Polygon(vertices, linewidth=2, edgecolor='k', facecolor='none')
        
        # Adding Polygon to the plot
        ax.add_patch(polygon)
