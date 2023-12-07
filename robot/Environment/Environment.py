class Environment:
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def point_in_obstacle(self, point):
        for obstacle in self.obstacles:
            if obstacle.contains_point(point):
                return True
        return False
