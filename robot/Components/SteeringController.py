from math import pi, sin, cos, atan2

class SteeringController:
    """
    A class to control the steering of a robot towards a specified goal.

    Attributes:
        _K_h (float): Gain for the steering control, affecting how quickly the robot turns towards the goal.
        max_steering (float): Maximum allowable steering angle to ensure the robot's turning is within realistic bounds.
    """
    def __init__(self):
        """
        Initializes the SteeringController with default steering gain and maximum steering angle.
        """
        # Gains for the steering controller
        self._K_h = 3  # Gain for the steering control
        self.max_steering = pi/2  # Maximum allowable steering angle
        
    def compute(self, goal, pose):
        """
        Computes the steering angle required to orient the robot towards the goal.

        The method calculates the desired orientation to the goal and adjusts the current steering angle to align 
        with this orientation. The steering angle is normalized and limited to ensure realistic robot movement.

        Parameters:
            goal (tuple): The target goal coordinates (x, y) towards which the robot should steer.
            pose (list): The current pose of the robot (x, y, theta).

        Returns:
            float: The calculated steering angle to align the robot towards the goal.
        """
        # Calculate the desired orientation towards the goal
        desired_theta = atan2(goal[1] - pose[1], goal[0] - pose[0])
        
        # Calculate the steering
        steering = desired_theta - pose[2]
        steering = self._K_h *atan2(sin(steering), cos(steering))  # Normalize the steering angle
        
        # Limit steering
        steering = sign(steering) * min(abs(steering), self.max_steering)
        
        return steering 
        
    def set_gains(self, K_steering):
        """
        Sets the gain for the steering controller.

        A higher gain results in more aggressive steering towards the goal.

        Parameters:
            K_steering (float): The new steering gain.
        """
        self._K_h = K_steering

def sign(x):
    """
    Determines the sign of a given number, indicating its positivity or negativity.

    Parameters:
        x (float): The number whose sign is to be determined.

    Returns:
        int: Returns 1 if the number is positive or zero, -1 if the number is negative.
    """
    if x >= 0:
        return 1
    else:
        return -1
