from math import pi, sin, cos, atan2

class SteeringController:
    def __init__(self):
        """
        Constructor: Initializes the steering controller with default gains.
        """
        # Gains for the steering controller
        self._K_h = 3  # Gain for the steering control
        self.max_steering = pi/2  # Maximum allowable steering angle
        
    def compute(self, goal, pose):
        """
        Computes the steering command based on the current pose and the goal position.
        """
        # Calculate the desired orientation towards the goal
        desired_theta = atan2(goal[1] - pose[1], goal[0] - pose[0])
        
        # Calculate the steering
        steering = desired_theta - pose[2]
        steering = self._K_h *atan2(sin(steering), cos(steering))  # Normalize the steering angle
        
        # Limit steering
        #steering = sign(steering) * min(abs(steering), self.max_steering)
        
        return steering 
        
    # Setter method to modify the gain of the controller
    def set_gains(self, K_steering):
        """
        Set the gain of the steering controller.
        """
        self._K_h = K_steering  # Set the steering gain

def sign(x):
    """
    Returns the sign of a given number.
    """
    if x >= 0:
        return 1
    else:
        return -1
