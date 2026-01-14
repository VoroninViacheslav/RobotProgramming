import os
import sys
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

sys.path.append(str(Path(__file__).parent))
from simulators.inverted_pendulum import run_simulation
from utils.plots import _plot_angle_comparison, _plot_control_signal


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Inverted pendulum control with PID",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default parameters (improved for stability):
  python src/main.py
  
  # Custom PID parameters:
  python src/main.py --kp-angle 200 --kd-angle 25 --ki-angle 2
    """
    )
    
    parser.add_argument("--headless", action="store_true",
                       help="Run simulation without GUI")
    parser.add_argument("--kp-angle", type=float, default=200.0,
                       help="Proportional gain for angle (default: 200.0)")
    parser.add_argument("--ki-angle", type=float, default=0.0,
                       help="Integral gain for angle (default: 2.0)")
    parser.add_argument("--kd-angle", type=float, default=25.0,
                       help="Derivative gain for angle (default: 25.0)")
    parser.add_argument("--kp-pos", type=float, default=15.0,
                       help="Proportional gain for position (default: 15.0)")
    parser.add_argument("--ki-pos", type=float, default=0.0,
                       help="Integral gain for position (default: 0.0)")
    parser.add_argument("--kd-pos", type=float, default=2.0,
                       help="Derivative gain for position (default: 2.0)")
    parser.add_argument("--duration", type=float, default=20.0,
                       help="Simulation duration in seconds (default: 30.0)")
    parser.add_argument("--initial-offset", type=float, default=0.2,
                       help="Initial angle offset from target in radians (default: 0.2 ≈ 11.5°)")
    parser.add_argument("--initial-offset-deg", type=float, default=None,
                       help="Initial angle offset from target in degrees (overrides --initial-offset)")
    parser.add_argument("--no-realtime", action="store_true",
                       help="Disable realtime visualization (run as fast as possible)")
    parser.add_argument("--slowdown", type=float, default=1.0,
                       help="Slowdown factor for visualization (1.0 = realtime, 2.0 = 2x slower) (default: 1.0)")
    
    args = parser.parse_args()
    
    initial_offset = args.initial_offset
    if args.initial_offset_deg is not None:
        initial_offset = np.radians(args.initial_offset_deg)
    
    model_path = os.path.join(os.path.dirname(__file__), "models", "cartpole.xml")
    
    run_simulation(
        model_path=model_path,
        kp_angle=args.kp_angle,
        ki_angle=args.ki_angle,
        kd_angle=args.kd_angle,
        kp_pos=args.kp_pos,
        ki_pos=args.ki_pos,
        kd_pos=args.kd_pos,
        duration=args.duration,
        headless=args.headless,
        initial_angle_offset=initial_offset,
        realtime=not args.no_realtime,
        slowdown=args.slowdown
    )
