from math import sqrt
#from Queue import Queue  # python 2.7, used in robot
from queue import Queue	  # python 3, used in simulation

class GoalController():
    def __init__(self):
        """
        Initialize the GoalController with necessary attributes.
        """
        self._goals = Queue()  # Queue to hold the sequence of goals
        self._goal = []  # Current goal
        self._visited = [] # Visited goals
        self.gap_goal = [] # Goal from the largest gap
        self._goals_reached = False  # Flag to check if goal is reached
        self._distance = 0  # Initializing the _distance attribute
        self.distance_accuracy = 0.1  # Accuracy threshold for reaching a goal

    def distance_to_goal(self, pose):
        """
        Calculate the Euclidean distance between the current pose and the goal.

        Parameters:
        - pose: The current position of the robot.

        Returns:
        - float: The Euclidean distance to the goal.
        """
        if not self._goal:
            return 0
        else:
            return sqrt((self._goal[0] - pose[0])**2 + (self._goal[1] - pose[1])**2)

    def update(self, pose, dt):
        """
        Update the goal and check whether it has been reached.

        Parameters:
        - pose: The current position of the robot.
        - dt: Time step.
        """
        self._update_current_goal(dt)
        
        if self._goals_reached:
            return

        self._distance = self.distance_to_goal(pose)
        
        if self._distance < self.distance_accuracy:
            self._visited.append(self._goal)
            self._goal_reached_action(dt)

    def _update_current_goal(self, dt):
        """
        Update the current goal if there is none.

        Parameters:
        - dt: Time step.
        """
        if not self._goal:
            self._goal_reached_action(dt)

    def _goal_reached_action(self, dt):
        """
        Actions to perform when a goal is reached.

        Parameters:
        - dt: Time step.
        """
        if self._goals.empty():
            self._goal = []
            self._goals_reached = True
        else:
            self._goal = self._goals.get()

    def add_goal(self, goal):
        """
        Add a new goal to the queue.

        Parameters:
        - goal: The new goal to be added.
        """
        self._goals.put(goal)

    def add_goals(self, goal_x, goal_y):
        """
        Add multiple goals to the queue.

        Parameters:
        - goal_x: List of x coordinates of the goals.
        - goal_y: List of y coordinates of the goals.
        """
        n = len(goal_x)
        for i in range(n):
            self.add_goal([goal_x[i], goal_y[i]])

    def all_goals_reached(self):
        """
        Check if all goals were reached.

        Returns:
        - bool: True if all goals have been reached, otherwise False.
        """
        return self._goals_reached
    
    def get_current_goal(self):
        """
            Return current goal
        """
        return self._goal
    
    def get_distance_to_goal(self):
        """
            Return distance to current goal
        """
        return self._distance
    
    def get_goals(self):
        """
            Return not visited goals
        """
        return self._goals
    
    def get_visited(self):
        """
            Return visited goals
        """
        return self._visited