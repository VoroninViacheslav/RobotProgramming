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
  python src/main.py --kp-angle 200 --kd-angle 20 --ki-angle 0 --kp-pos 3 --kd-pos 1.5
  LQR
  python src/main.py --controller lqr --duration 5 --initial-offset 0.1
    """
    )
    
    parser.add_argument("--headless", action="store_true",
                       help="Run simulation without GUI")
    parser.add_argument("--kp-angle", type=float, default=200.0,
                       help="Proportional gain for angle (default: 200.0)")
    parser.add_argument("--ki-angle", type=float, default=0.0,
                       help="Integral gain for angle (default: 2.0)")
    parser.add_argument("--kd-angle", type=float, default=20.0,
                       help="Derivative gain for angle (default: 25.0)")
    parser.add_argument("--kp-pos", type=float, default=3.0,
                       help="Proportional gain for position (default: 15.0)")
    parser.add_argument("--ki-pos", type=float, default=0.0,
                       help="Integral gain for position (default: 0.0)")
    parser.add_argument("--kd-pos", type=float, default=1.5,
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
    parser.add_argument("--controller", type=str, default="pid", choices=["pid", "lqr"],
                       help="Controller type: 'pid' or 'lqr' (default: pid)")
    
    args = parser.parse_args()
    
    initial_offset = args.initial_offset
    if args.initial_offset_deg is not None:
        initial_offset = np.radians(args.initial_offset_deg)
    
    K = None
    if args.controller.lower() == "lqr":
        K = np.array([-31.6227766016845,	-61.2570308511974,	-577.941806517827,	-326.292325681899])

    
    model_path = os.path.join(os.path.dirname(__file__), "models", "cartpole.xml")
    
    run_simulation(
        model_path=model_path,
        controller_type=args.controller,
        kp_angle=args.kp_angle,
        ki_angle=args.ki_angle,
        kd_angle=args.kd_angle,
        kp_pos=args.kp_pos,
        ki_pos=args.ki_pos,
        kd_pos=args.kd_pos,
        K=K,
        duration=args.duration,
        headless=args.headless,
        initial_angle_offset=initial_offset,
        realtime=not args.no_realtime,
        slowdown=args.slowdown
    )
