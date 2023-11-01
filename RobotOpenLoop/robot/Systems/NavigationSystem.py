from math import sqrt

class NavigationSystem():
    def __init__(self, odometer, pid, steering_controller, proportional_controller, goal_controller):
        """
        Constructor: Initializes the NavigationSystem with necessary controllers and components.
        
        Parameters:
        - odometer: The odometer component for pose and velocity.
        - pid: The PID controller for velocity.
        - steering_controller: The steering controller for direction.
        - proportional_controller: The proportional controller as an alternative for PID.
        - goal_controller: The goal controller managing navigation goals.
        """
        
        # Modules
        self._odometer = odometer
        self._pid = pid
        self._steering_controller = steering_controller
        self._proportional_controller = proportional_controller
        self._goal_controller = goal_controller
        
        self.type = "pid"  # Default type of controller to use for velocity.

    def update(self, dt):
        """
        Update the Navigation System, including goal, steering, and velocity controllers.
        
        Parameters:
        - dt: The time step.
        """
        # Retrieve general stats from the odometer.
        pose = self._odometer.get_pose()  
        vel = self._odometer.get_vel()  
        cruise_vel = self._odometer.cruise_vel  
        max_vel = self._odometer.max_vel  
        
        # Update the goal controller with the current pose.
        self._goal_controller.update(pose, dt)
        
        # Check if all goals have been reached, then reset the odometer.
        if self._goal_controller.all_goals_reached():
            self._odometer.reset()
            return
        
        # Calculate steering command based on the current goal.
        goal = self._goal_controller.get_current_goal()
        steering = self._steering_controller.compute(goal, pose)

        # Calculate velocity command based on the chosen control strategy and update the odometer.
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