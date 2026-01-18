import numpy as np
from controllers.pid_controller import PIDController

class PidPendulumController:

    def __init__(self, kp_angle: float = 0.0, ki_angle: float = 0.0, kd_angle: float = 0.0,
                 kp_pos: float = 0.0, ki_pos: float = 0.0, kd_pos: float = 0.0,
                 output_limit: float = 300.0, integral_limit: float = 5.0,
                 target_angle: float= 0.0, target_pos: float = 0.0):

        self.target_angle = target_angle
        self.target_pos = target_pos

        self.integral = 0.0
        self.prev_error_angle = 0.0
        self.prev_error_pos = 0.0

        self.output_limit = output_limit

        self.pid_controller_pole = PIDController(kp_angle, ki_angle, kd_angle, output_limit, integral_limit)
        self.pid_controller_cart = PIDController(kp_pos, ki_pos, kd_pos, output_limit, integral_limit)

    def compute(self, pole_angle: float, dt: float, cart_pos: float) -> float:

        error_angle = pole_angle - self.target_angle
        error_angle = np.arctan2(np.sin(error_angle), np.cos(error_angle))
        error_pos = cart_pos - self.target_pos

        pole_control = self.pid_controller_pole.compute(error_angle, dt)
        cart_control = self.pid_controller_cart.compute(error_pos, dt)
        output = np.clip(pole_control + cart_control, -self.output_limit, self.output_limit)

        self.prev_error_angle = error_angle
        self.prev_error_pos = error_pos

        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error_angle = 0.0
        self.prev_error_pos = 0.0
