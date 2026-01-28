#!/usr/bin/env python3
"""
Simulate and plot continuous-time LQR closed-loop response
for the balancing robot described by LQR_bot_data.json.

also plots the control torque u(t) = -Kx(t)
in a second subplot beneath the state responses.

Usage:
    ./plot_TWR_response.py path/to/LQR_bot_data.json

"""

import sys, json
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import StateSpace, lsim


def simulate_closed_loop(data, t_final=1.5, n_points=2000):
    """Simulate xdot = (A - BK)x and compute control torque u = -Kx."""
    A = np.array(data["A_cont"])
    B = np.array(data["B_cont"])
    K = np.array(data["K_cont"])
    A_cl = A - B @ K

    # Continuous state-space model
    C = np.eye(4)
    D = np.zeros((4, 1))
    sys = StateSpace(A_cl, B, C, D)

    # Initial condition: 0.1 rad tilt (~5.7°)
    x0 = np.array([0.1, 0, 0, 0])
    t = np.linspace(0, t_final, n_points)
    u = np.zeros_like(t)
    t, y, x = lsim(sys, U=u, T=t, X0=x0)

    # Compute control torque at each step
    u_cmd = - (K @ x.T).flatten()
    return t, y, u_cmd


def plot_response(t, y, u_cmd):
    """Plot state trajectories and control torque."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # ---- Top: state trajectories ----
    labels = [
        "Body angle θ (rad)",
        "Body angular rate θ̇ (rad/s)",
        "Wheel position x (m)",
        "Wheel velocity ẋ (m/s)"
    ]
    for i in range(4):
        ax1.plot(t, y[:, i], label=labels[i])
    ax1.set_ylabel("State value")
    ax1.set_title("LQR Closed-Loop Response (small initial tilt)")
    ax1.legend()
    ax1.grid(True)

    # ---- Bottom: control torque ----
    ax2.plot(t, u_cmd, color="black", label="Control torque u (N·m)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Torque (N·m)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    if len(sys.argv) != 2:
        print("Usage: ./plot_LQR_response.py path/to/LQR_bot_data.json")
        sys.exit(1)

    json_path = sys.argv[1]
    with open(json_path, "r") as f:
        data = json.load(f)

    print(f"Loaded {json_path}")
    t, y, u_cmd = simulate_closed_loop(data)
    plot_response(t, y, u_cmd)


if __name__ == "__main__":
    main()
