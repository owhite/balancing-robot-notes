#!/usr/bin/env python3
"""
Plot discrete-time LQR closed-loop response from pendulum_LQR_data.json

Usage:
    python plot_lqr_response.py pendulum_LQR_data.json [sample_period_s]

If no sample period is provided, defaults to Ts = 1/500 s (500 Hz).
"""

import json
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm


def simulate_discrete_response(A, B, K, Ts, x0, t_final):
    """Simulate x[k+1] = (A_d - B_d K) x[k] for a given sample time Ts."""
    # Discretize continuous-time system
    M = np.block([[A, B],
                  [np.zeros((1, 3))]])
    Md = expm(M * Ts)
    Ad = Md[:2, :2]
    Bd = Md[:2, 2:]

    steps = int(t_final / Ts)
    x = np.zeros((steps, 2))
    u = np.zeros(steps)
    x[0] = x0

    for k in range(steps - 1):
        u[k] = -K @ x[k]
        x[k + 1] = (Ad - Bd @ K) @ x[k]

    t = np.arange(steps) * Ts
    return t, x, u


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_lqr_response.py pendulum_LQR_data.json [sample_period_s]")
        sys.exit(1)

    json_path = sys.argv[1]
    Ts = float(sys.argv[2]) if len(sys.argv) > 2 else 1 / 500  # default 500 Hz loop

    # Load parameters from JSON
    with open(json_path, "r") as f:
        data = json.load(f)

    A = np.array(data["A_matrix_continuous"])
    B = np.array(data["B_matrix_continuous"])
    K = np.array(data["K_gain_continuous"])
    m = data["mass_kg"]
    I = data["moment_of_inertia_kg_m2"]
    r = data["r_m"]

    print("\n📘 Loaded parameters from JSON:")
    print(f"  A = {A}")
    print(f"  B = {B}")
    print(f"  K = {K}")
    print(f"  Ts = {Ts*1000:.2f} ms")
    print(f"  m = {m:.4f} kg, I = {I:.6e} kg·m², r = {r:.4f} m")
    # Initial condition (5.7°)
    x0 = np.array([0.1, 0.0])
    t, x, u = simulate_discrete_response(A, B, K, Ts, x0, t_final=3.0)

    # Plot states
    plt.figure(figsize=(7, 4))
    plt.plot(t, x[:, 0], label="Angle θ [rad]")
    plt.plot(t, x[:, 1], label="Angular velocity θ̇ [rad/s]")
    plt.xlabel("Time [s]")
    plt.ylabel("State")
    plt.title(f"LQR Closed-Loop Response (Discrete, {1/Ts:.0f} Hz loop)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # Optional torque plot
    plt.figure(figsize=(7, 3))
    plt.plot(t, u, label="Control torque [N·m]")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [N·m]")
    plt.title("Control Effort")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
