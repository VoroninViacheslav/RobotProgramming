import numpy as np


class LqrPendulumController:
    
    def __init__(self, K: np.ndarray = None, 
                 target_angle: float = 0.0, target_pos: float = 0.0,
                 output_limit: float = 900.0):
        
        self.K = np.array(K)
        self.target_angle = target_angle
        self.target_pos = target_pos
        self.output_limit = output_limit
        self.prev_angle = 0.0
        self.prev_pos = 0.0

    def compute(self, pole_angle: float, cart_pos: float, dt: float) -> float:
        
        state = np.array([cart_pos - self.target_pos,
                          (cart_pos - self.prev_pos)/dt,
                          pole_angle - self.target_angle,
                          (pole_angle - self.prev_angle)/dt])

        control = -np.dot(self.K, state)
        control = np.clip(control, -self.output_limit, self.output_limit)
        
        self.prev_angle = pole_angle
        self.prev_pos = cart_pos

        return control
    
    def reset(self):
        self.prev_error_angle = 0.0
        self.prev_error_pos = 0.0
