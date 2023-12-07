class ProportionalController:
    def __init__(self):
        """
        Constructor: Initializes the proportional controller with default gains and resets internal states.
        """
        # Gains for the proportional controller
        self._K_p = .8  # Proportional gain
        self._K_i = 0.01  # Integral gain
        
        # Internal states for the controller
        self._prev_integral = 0  # Previous integral term
        self._prev_error = 0  # Previous error term
        
    def update(self, distance, dt):
        """
        Updates the controller output based on the current error and integral of the error.
        """
        error = distance  # Current error
        integral = self._prev_integral + error * dt  # Integral of the error
        
        # Calculate the output based on the proportional and integral terms
        output = self._K_p * error + self._K_i * integral
        
        # Update the internal states for the next iteration
        self._prev_error = error
        self._prev_integral = integral
        
        return output  # Return the controller output
        
    def reset(self):
        """
        Resets the internal states of the controller.
        """
        self._prev_integral = 0
        self._prev_error = 0
        
    # Setter methods to modify the gains of the controller
    def set_gains(self, K_p, K_i):
        """
        Set the gains of the proportional controller.
        """
        self._K_p = K_p  # Set the proportional gain
        self._K_i = K_i  # Set the integral gain
