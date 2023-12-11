class PidController:
    """
    A class that implements a PID controller, commonly used in control systems to adjust a process to a desired setpoint.

    Attributes:
        _K_p            (float): Proportional gain, determines the reaction to the current error.
        _K_i            (float): Integral gain, determines the reaction based on the sum of recent errors.
        _K_d            (float): Derivative gain, determines the reaction to the rate of change of the error.
        _prev_integral  (float): The accumulated integral of the error up to the previous update.
        _prev_error     (float): The error value from the previous update.
    """
    def __init__(self):
        """
        Initializes the PID controller with default gain values and resets internal states.
        """
        self._K_p = 0.1  
        self._K_i = 0.001  
        self._K_d = 0.03
        self._prev_integral = 0  
        self._prev_error = 0 
        
    def update(self, cruise_vel, vel, dt):
        """
        Updates the PID controller output based on the current error (difference between desired and actual values),
        the integral of the error over time, and the rate of change of the error.

        Parameters:
            cruise_vel  (float): The desired cruise velocity (setpoint).
            vel         (float): The current velocity (process variable).
            dt          (float): The time step between updates.

        Returns:
            float: The updated control output that should be applied to the process.
        """
        # Calculate errors
        error = cruise_vel - vel
        integral = self._prev_integral + error*dt
        derivative = (error - self._prev_error)/dt
        
        # Calculate the PID output
        output = vel + self._K_p * error + self._K_i * integral + self._K_d * derivative
        
        # Update the internal states for the next iteration
        self._prev_error = error
        self._prev_integral = integral
        
        return output
        
    def reset(self):
        """
        Resets the internal states (integral, error, and output) of the PID controller.
        """
        self._prev_integral = 0
        self._prev_error = 0
        
    def set_gains(self, K_p, K_i, K_d):
        """
        Sets the gains for the PID controller.

        Parameters:
            K_p (float): Proportional gain.
            K_i (float): Integral gain.
            K_d (float): Derivative gain.
        """
        self._K_p = K_p  
        self._K_i = K_i  
        self._K_d = K_d
