class PidController:
    def __init__(self):
        """
        Constructor: Initializes the PID controller with default gains and internal states.
        """
        # Gains for the PID controller
        self._K_p = 0.1  # Proportional gain
        self._K_i = 0.001  # Integral gain
        self._K_d = 0.03  # Derivative gain
        
        # Internal states for the PID controller
        self._prev_integral = 0  # Previous integral term
        self._prev_error = 0  # Previous error term
        self._prev_output = 0  # Previous output term
        
    def update(self, cruise_vel, vel, dt):
        """
        Updates the PID controller output based on the current error, integral, and derivative of the error.
        """
        error = cruise_vel - vel  # Calculate the error
        integral = self._prev_integral + error*dt  # Calculate the integral of the error
        derivative = (error - self._prev_error)/dt  # Calculate the derivative of the error
        # Calculate the PID output
        output = vel + self._K_p * error + self._K_i * integral + self._K_d * derivative
        
        # Update the internal states for the next iteration
        self._prev_error = error
        self._prev_integral = integral
        self._prev_output = output
        
        return output  # Return the PID output
        
    def reset(self):
        """
        Resets the internal states of the PID controller.
        """
        self._prev_integral = 0
        self._prev_error = 0
        self._prev_output = 0
        
    # Setters to modify the gains of the PID controller
    def set_gains(self, K_p, K_i, K_d):
        """
        Set the gains of the PID controller.
        """
        self._K_p = K_p  # Set the proportional gain
        self._K_i = K_i  # Set the integral gain
        self._K_d = K_d  # Set the derivative gain
