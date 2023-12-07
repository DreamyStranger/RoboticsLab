import matplotlib.patches as patches

class RectangleObstacle:
    def __init__(self, x, y, width, height):
        """Initialize a rectangle with the given parameters."""
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def contains_point(self, point):
        """Check if the rectangle contains the given point."""
        x, y = point
        return self.x < x < self.x + self.width and self.y < y < self.y + self.height
    
    def draw(self, ax):
        """Draw the rectangle on the given axis."""
        # Defining vertices of the polygon (rectangle)
        vertices = [(self.x, self.y), (self.x + self.width, self.y), 
                    (self.x + self.width, self.y + self.height), (self.x, self.y + self.height)]
        
        # Creating a Polygon patch
        polygon = patches.Polygon(vertices, linewidth=2, edgecolor='k', facecolor='none')
        
        # Adding Polygon to the plot
        ax.add_patch(polygon)
