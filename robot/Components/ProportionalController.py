class ProportionalController:
    """
    A class that implements a Proportional-Integral (PI) controller, which is used to adjust a process 
    based on the current error and the accumulation of past errors.

    Attributes:
        _K_p (float): Proportional gain, determining the reaction to the current error.
        _K_i (float): Integral gain, determining the reaction based on the sum of recent errors.
        _prev_integral (float): The accumulated integral of the error up to the previous update.
        _prev_error (float): The error value from the previous update.
    """
    def __init__(self):
        """
        Initializes the Proportional-Integral controller with default gain values and resets internal states.
        """
        self._K_p = .8 
        self._K_i = 0.01
        self._prev_integral = 0 
        self._prev_error = 0 
        
    def update(self, distance, dt):
        """
        Updates the controller output based on the current error (difference between desired and actual values) 
        and the integral of the error over time.

        Parameters:
            distance (float): The current error or distance from the target.
            dt (float): The time step between updates.

        Returns:
            float: The updated control output that should be applied to the process.
        """
        # Calculate errors
        error = distance
        integral = self._prev_integral + error * dt
        
        # Calculate the output
        output = self._K_p * error + self._K_i * integral
        
        # Update the internal states for the next iteration
        self._prev_error = error
        self._prev_integral = integral
        
        return output
        
    def reset(self):
        """
        Resets the internal states (integral and error) of the controller.
        """
        self._prev_integral = 0
        self._prev_error = 0
        
    # Setter methods to modify the gains of the controller
    def set_gains(self, K_p, K_i):
        """
        Sets the gains for the Proportional-Integral controller.

        Parameters:
            K_p (float): Proportional gain.
            K_i (float): Integral gain.
        """
        self._K_p = K_p
        self._K_i = K_i 
