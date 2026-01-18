import os
import numpy as np
import mujoco
import mujoco.viewer


class MuJoCoSimulator: 
    def __init__(self, model_path: str):

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
    
    def reset(self, cart_pos: float = 0.0, pole_angle: float = 0.0):

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = cart_pos
        self.data.qpos[1] = pole_angle
    
    def step(self, control: float):

        self.data.ctrl[0] = control 
        mujoco.mj_step(self.model, self.data)
    
    def get_state(self) -> dict:

        return {
            'cart_pos': self.data.qpos[0],
            'cart_vel': self.data.qvel[0],
            'pole_angle': self.data.qpos[1],
            'pole_vel': self.data.qvel[1]
        }
    
    def render(self, headless: bool = False):

        if not headless:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._setup_camera()
    
    def _setup_camera(self):

        if self.viewer is not None:
            self.viewer.cam.lookat[:] = [0, 0, 0.5]
            self.viewer.cam.distance = 8.0
            self.viewer.cam.azimuth = 90.0
            self.viewer.cam.elevation = -10.0
    
    def update_viewer(self):

        if self.viewer is not None:
            try:
                self.viewer.sync()
                return True
            except:
                return False
        return True
    
    def close(self):

        if self.viewer is not None:
            try:
                self.viewer.close()
            except:
                pass
