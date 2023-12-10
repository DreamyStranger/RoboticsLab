from math import sqrt
#from Queue import Queue  # python 2.7, used in robot
from queue import Queue	  # python 3, used in simulation

class GoalController():
    """
    A class for managing navigation goals in a robotic system.

    Attributes:
        _goals (Queue): A queue of goals to be reached by the robot.
        _goal (list): The current goal the robot is navigating towards.
        _visited (list): A list of goals that have been visited/reached.
        gap_goal (list): The current goal derived from the largest gap detected.
        _goals_reached (bool): Flag indicating whether all goals have been reached.
        _distance (float): The current distance to the goal.
        distance_accuracy (float): The accuracy threshold for reaching a goal.
    """
    def __init__(self):
        """
        Initializes the GoalController with default settings and empty goal structures.
        """
        self._goals = Queue() 
        self._goal = []
        self._visited = []
        self.gap_goal = [] 
        self._goals_reached = False 
        self._distance = 0  
        self.distance_accuracy = 0.1 

    def distance_to_goal(self, pose):
        """
        Calculates the Euclidean distance between the robot's current position and the goal.

        Parameters:
            pose (tuple): The current position of the robot (x, y, theta).

        Returns:
            float: The Euclidean distance to the current goal.
        """
        if not self._goal:
            return 0
        else:
            return sqrt((self._goal[0] - pose[0])**2 + (self._goal[1] - pose[1])**2)

    def update(self, pose, dt):
        """
        Updates the status of the current goal. Checks if the goal has been reached and updates the goal queue accordingly.

        Parameters:
            pose (tuple): The current position of the robot (x, y, theta).
            dt (float): The time step for updating the goal status.
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
        Internal method to update the current goal if there is none.

        Parameters:
            dt (float): The time step for updating the goal status.
        """
        if not self._goal:
            self._goal_reached_action(dt)

    def _goal_reached_action(self, dt):
        """
        Internal method to perform actions when a goal is reached. This includes updating the visited list and fetching the next goal from the queue.

        Parameters:
            dt (float): The time step for updating the goal status.
        """
        if self._goals.empty():
            self._goal = []
            self._goals_reached = True
        else:
            self._goal = self._goals.get()

    def add_goal(self, goal):
        """
        Adds a new goal to the queue.

        Parameters:
            goal (list): The new goal coordinates (x, y) to be added.
        """
        self._goals.put(goal)

    def add_goals(self, goal_x, goal_y):
        """
        Adds multiple goals to the queue.

        Parameters:
            goal_x (list): List of x coordinates of the goals.
            goal_y (list): List of y coordinates of the goals.
        """
        n = len(goal_x)
        for i in range(n):
            self.add_goal([goal_x[i], goal_y[i]])

    def all_goals_reached(self):
        """
        Checks whether all goals in the queue have been reached.

        Returns:
            bool: True if all goals have been reached, False otherwise.
        """
        return self._goals_reached
    
    def get_current_goal(self):
        """
        Retrieves the current goal coordinates.

        Returns:
            list: The current goal coordinates (x, y).
        """
        return self._goal
    
    def get_distance_to_goal(self):
        """
        Retrieves the current distance to the goal.

        Returns:
            float: The current distance to the goal.
        """
        return self._distance
    
    def get_goals(self):
        """
        Retrieves the queue of goals that have not been visited.

        Returns:
            Queue: The queue of unvisited goals.
        """
        return self._goals
    
    def get_visited(self):
        """
        Retrieves the list of goals that have been visited.

        Returns:
            list: The list of visited goals.
        """
        return self._visited
    
    def reset(self):
        """
        Resets the goal controller. This includes clearing the current goal, the queue of goals, 
        the gap goal, and resetting the flags and distances associated with goal management.

        After calling this method, the goal controller will have no current goal, no goals in the queue, and the gap 
        goal will be cleared. The system will be ready to accept and process new goals.
        """
        self._goal = []
        self._goals = Queue()
        self.gap_goal = []
        self._goals_reached = False
        self._distance = 0