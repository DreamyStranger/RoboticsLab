import matplotlib.patches as patches

class RectangleObstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def contains_point(self, point):
        x, y = point
        return self.x < x < self.x + self.width and self.y < y < self.y + self.height
    
    def draw(self, ax):
        # Defining vertices of the polygon (rectangle)
        vertices = [(self.x, self.y), (self.x + self.width, self.y), 
                    (self.x + self.width, self.y + self.height), (self.x, self.y + self.height)]
        
        # Creating a Polygon patch
        polygon = patches.Polygon(vertices, linewidth=2, edgecolor='k', facecolor='none')
        
        # Adding Polygon to the plot
        ax.add_patch(polygon)

    """
    def calculate_intersections(self, x, y, angle):
        intersections = []
        corners = [
            (self.x, self.y),
            (self.x + self.width, self.y),
            (self.x, self.y + self.height),
            (self.x + self.width, self.y + self.height)
        ]
        edges = [
            (corners[0], corners[1]),
            (corners[1], corners[3]),
            (corners[3], corners[2]),
            (corners[2], corners[0]),
        ]

        for edge in edges:
            intersection = self._calculate_line_intersection(edge, x, y, angle)
            if intersection:
                intersections.append(intersection)

        return intersections

    def _calculate_line_intersection(self, edge, x, y, angle):
        # Implement the logic to find the intersection point between a line
        # defined by edge and a ray cast from (x, y) at a certain angle.
        # You might use line-line intersection formulas for this.
        # If there's an intersection, return the intersection point as a tuple (x, y).
        # Otherwise, return None.
        pass
    """
