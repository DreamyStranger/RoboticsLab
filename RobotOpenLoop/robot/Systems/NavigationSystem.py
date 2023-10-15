from math import sqrt
#from Queue import Queue  # python 2.7, used in robot
from queue import Queue	  # python 3, used in simulation

class NavigationSystem():
    def __init__(self, odometer, pid, steering_controller, proportional_controller):
        """
        Constructor: Initializes the NavigationSystem with necessary controllers and components.
        """
        # Modules
        self._odometer = odometer  # Odometer component for pose and velocity
        self._pid = pid  # PID controller for velocity
        self._steering_controller = steering_controller  # Steering controller for direction
        self._proportional_controller = proportional_controller  # Proportional controller as an alternative for PID

        # Destination
        self._goals = Queue()  # Queue to hold the sequence of goals
        self._goal = []  # Current goal
        self._distance = 0  # Distance to the current goal
        self._distance_accuracy = 0.1  # Accuracy threshold for reaching a goal
        self.type = "pid"  # Type of controller to use for velocity ("pid" or proportional)
        self._goal_reached = False  # Flag to check if goal is reached

    def distance_to_point(self, point):
        """
        Calculates the Euclidean distance between robot's position
        and a given point, and returns it.
        """
        pose = self._odometer.get_pose()  # Get the current pose of the robot
        # Calculate and return the Euclidean distance to the given point
        self._distance = sqrt((point[0] - pose[0])**2 + (point[1] - pose[1])**2)
        return self._distance

    def update(self, dt):
        """
        Main update loop: It updates the goal, calculates the steering
        and velocity commands, and updates the robot's configuration.
        """
        # Update the current goal
        self.update_goal(dt)
        if self._goal_reached:
            return

        # Get general stats
        goal = self._goal  # Current goal
        pose = self._odometer.get_pose()  # Current pose
        vel = self._odometer.get_vel()  # Current velocity
        cruise_vel = self._odometer.cruise_vel  # Cruise velocity
        max_vel = self._odometer.max_vel  # Maximum allowable velocity

        # Calculate distance to the current goal
        distance = self.distance_to_point(goal)
        # Check if the goal is reached, and update the goal if necessary
        if abs(distance) < self._distance_accuracy:
            self.goal_reached_action(dt)
            
        # Calculate steering command based on the current goal
        steering = self._steering_controller.compute(goal, pose)

        # Calculate velocity command based on the chosen controller type
        vel = self.calculate_velocity(cruise_vel, vel, dt, distance, max_vel)

        # Update the robot's configuration
        self._odometer.set_vel(vel)
        self._odometer.set_steering(steering)
        self._odometer.update(dt)

    def update_goal(self, dt):
        """
        Updates the current goal. If no goal is available, it sets goal_reached to True.
        """
        if not self._goal:  # Check if there is a current goal
            self.no_goal_action(dt)

    def no_goal_action(self, dt):
        """
        Action to perform when there's no current goal.
        """
        if self._goals.empty():
            self._odometer.reset()
            self._odometer.update(dt)
            self._goal_reached = True
        else:
            self._goal = self._goals.get()

    def goal_reached_action(self, dt):
        """
        Action to perform when a goal is reached.
        """
        if self._goals.empty():
            self._goal = []
            self._odometer.reset()
            self._odometer.update(dt)
        else:
            self._goal = self._goals.get()

    def calculate_velocity(self, cruise_vel, vel, dt, distance, max_vel):
        """
        Calculate the velocity command based on the selected control strategy.
        """
        if self.type == "pid":
            vel = self._pid.update(cruise_vel, vel, dt)
        else:
            vel = self._proportional_controller.update(distance, dt)
        return min(abs(vel), max_vel)

    def add_goal(self, goal):
        """
        Adds a new goal to the queue.
        """
        self._goals.put(goal)

    def add_goals(self, goal_x, goal_y):
        """
        Adds multiple goals to the queue.
        """
        n = len(goal_x)
        for i in range(n):
            self.add_goal([goal_x[i], goal_y[i]])
    
    @property
    def is_goal_reached(self):
        """
        Checks if all goals were reached.
        Returns True if all goals have been reached, otherwise False.
        """
        return self._goal_reached
