import numpy as np

class PIDController:
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limit: float = 1.0, integral_limit: float = 10.0):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        
        self.integral = 0.0
        self.prev_error = 0.0
    
    def compute(self, error: float, dt: float) -> float:

        proportional = self.kp * error
        
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        integral_term = self.ki * self.integral

        derivative = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0        
        
        output = proportional + integral_term + derivative
        output = np.clip(-self.output_limit, self.output_limit, output)
        
        self.prev_error = error
        return output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
