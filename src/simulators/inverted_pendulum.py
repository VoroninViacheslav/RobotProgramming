import numpy as np
import time
from controllers.pendulum_controller import PidPendulumController
from controllers.lqr_controller import LqrPendulumController
from simulators.mujoco_simulator import MuJoCoSimulator
from utils.plots import _plot_angle_comparison, _plot_control_signal

def run_simulation(model_path: str, controller_type: str = "pid",
                   kp_angle: float = 1.0, ki_angle: float = 0.0, 
                   kd_angle: float = 0.0, kp_pos: float = 0.0, ki_pos: float = 0.0,
                   kd_pos: float = 0.0, K: np.ndarray = None, duration: float = 30.0, 
                   headless: bool = False, initial_angle_offset: float = 0.2,
                   realtime: bool = True, slowdown: float = 1.0):

    simulator = MuJoCoSimulator(model_path)
    
    target_angle = 0.0
    target_position = 0.0
    
    initial_angle = initial_angle_offset
    simulator.reset(cart_pos=0.0, pole_angle=initial_angle)
    
    if not headless:
        simulator.render(headless=False)
        time.sleep(0.5)
    
    if controller_type.lower() == "lqr":
        controller = LqrPendulumController(
            K=K, target_angle=target_angle, target_pos=target_position
        )
        print(f"Controller: LQR with K = {controller.K}")
        print(f"LQR target angle = {np.degrees(target_angle):.1f}° (0 radians, top position)")
    else:
        controller = PidPendulumController(
            kp_angle=kp_angle, ki_angle=ki_angle, kd_angle=kd_angle,
            kp_pos=kp_pos, ki_pos=ki_pos, kd_pos=kd_pos,
            target_angle=target_angle, target_pos=target_position
        )
        print(f"PID parameters - Angle: Kp={kp_angle}, Ki={ki_angle}, Kd={kd_angle}")
        print(f"PID parameters - Position: Kp={kp_pos}, Ki={ki_pos}, Kd={kd_pos}")
    
    dt = simulator.model.opt.timestep
    
    states = []
    controls = []
    errors = []
    
    num_steps = int(duration / dt)
    
    print(f"Starting simulation for {duration} seconds...")
    print(f"Target: pole angle = {np.degrees(target_angle):.1f}°, cart position = {target_position:.3f} m")
    print(f"Initial: pole angle = {np.degrees(initial_angle):.1f}° (offset: {np.degrees(initial_angle_offset):.1f}°)")
    if not headless and realtime:
        print(f"Visualization: realtime (slowdown: {slowdown}x)")
    
    start_time = time.time()
    prev_error = 0
    for step in range(num_steps):
        state = simulator.get_state()
        states.append(state.copy())
        
        angle_error = target_angle - state['pole_angle']
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        errors.append(angle_error)
        
        prev_error = angle_error

        control = controller.compute(
            pole_angle=state['pole_angle'],
            cart_pos=state['cart_pos'],
            dt=dt
        )

        controls.append(control)
        
        simulator.step(control)
        
        if not headless:
            if not simulator.update_viewer():
                print("\nViewer closed by user. Stopping simulation...")
                break
            
            if realtime:
                elapsed = time.time() - start_time
                expected_time = (step + 1) * dt * slowdown
                sleep_time = expected_time - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
    
    if not headless:
        print("\nSimulation completed. Viewer will remain open for 5 seconds...")
        time.sleep(5.0)
    
    simulator.close()
    
    states_array = np.array([(s['cart_pos'], s['pole_angle']) for s in states])
    errors_array = np.array(errors)
    controls_array = np.array(controls)
    
    time_array = np.arange(len(states)) * dt
    angles_array = np.array([s['pole_angle'] for s in states])
    target_angles_array = np.full_like(angles_array, target_angle)
    
    print(f"\nSimulation statistics:")
    print(f"Final cart position: {states[-1]['cart_pos']:.4f} m")
    print(f"Final pole angle: {np.degrees(states[-1]['pole_angle']):.2f} deg")
    print(f"Final angle error: {np.degrees(errors[-1]):.2f} deg")
    print(f"RMSE angle error: {np.sqrt(np.mean(errors_array**2)):.4f} rad")
    print(f"Max control signal: {np.max(np.abs(controls_array)):.4f}")
    
    _plot_angle_comparison(time_array, angles_array, target_angles_array, headless)
    _plot_control_signal(time_array, controls_array, headless)
    
    return states_array, errors_array, controls_array