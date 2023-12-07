from math import sqrt

class NavigationSystem():
    """
    A class that represents the navigation system of a robot. This system integrates various controllers 
    and components to manage the robot's navigation towards goals, including steering and velocity adjustments.

    Attributes:
        _odometer (Odometer): The odometer component of the robot for tracking its pose.
        _pid (PidController): The PID controller for velocity control.
        _steering_controller (SteeringController): The steering controller for direction control.
        _proportional_controller (ProportionalController): The proportional controller as an alternative for PID control.
        _goal_controller (GoalController): The goal management component of the robot.
        type (str): The default type of velocity controller to use ("pid" or other types).
    """
    def __init__(self, robot):
        """
        Initializes the NavigationSystem with necessary controllers and components from the robot.

        Parameters:
            robot (Robot): The robot instance containing the necessary components for navigation.
        """
        self._odometer = robot.odometer
        self._pid = robot.pid
        self._steering_controller = robot.steering_controller
        self._proportional_controller = robot.proportional_controller
        self._goal_controller = robot.goal_controller
        
        self.type = "pid"

    def update(self, dt):
        """
        Updates the navigation system based on the current robot pose, goal status, and time step. 
        This includes updating goal status, calculating steering commands, and adjusting velocity.

        Parameters:
            dt (float): The time step for updating the navigation system.
        """
        # Retrieve current configuration from the Odometer.
        pose = self._odometer.get_pose()  
        vel = self._odometer.get_vel()  
        cruise_vel = self._odometer.cruise_vel  
        max_vel = self._odometer.max_vel  
        
        # Update the Goal Controller.
        self._goal_controller.update(pose, dt)
        
        # All goals reached.
        if self._goal_controller.all_goals_reached():
            self._odometer.reset()
            return
        
        # Compute steering.
        gap_goal = self._goal_controller.gap_goal
        goal = self._goal_controller.get_current_goal()
        if not gap_goal:
            steering = self._steering_controller.compute(goal, pose)
        else:
            steering = self._steering_controller.compute(gap_goal, pose)

        # Compute velocity.
        distance = self._goal_controller.get_distance_to_goal()
        vel = self.calculate_velocity(cruise_vel, vel, dt, distance, max_vel)

        # Update the robot's configuration.
        self._odometer.set_vel(vel)
        self._odometer.set_steering(steering)
        self._odometer.update(dt)

    def calculate_velocity(self, cruise_vel, vel, dt, distance, max_vel):
        """
        Calculate the velocity command based on the selected control strategy (PID or Proportional).
        
        Parameters:
        - cruise_vel: The desired cruise velocity.
        - vel: The current velocity.
        - dt: The time step.
        - distance: The distance to the current goal.
        - max_vel: The maximum allowable velocity.
        
        Returns:
        - float: The calculated velocity command.
        """
        if self.type == "pid":
            vel = self._pid.update(cruise_vel, vel, dt)
        else:
            vel = self._proportional_controller.update(distance, dt)
        return min(abs(vel), max_vel)