#!/usr/bin/env python3
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# --- Load your measured data (from one sinewave run) ---
with open("results_single_sine.json") as f:
    data = json.load(f)

t_vals = np.array([s["t"] * 1e-6 for s in data["samples"]])  # µs → s
torque_vals = np.array([s["torque"] * 0.056 * 30 for s in data["samples"]])  # convert back to N·m
pos_measured = np.array([s["pos"] for s in data["samples"]])

# --- Model parameters (from fitting) ---
J = 0.000719
b = 0.001654
k = 0.02137

# --- Interpolate torque for arbitrary t ---
from scipy.interpolate import interp1d
tau = interp1d(t_vals, torque_vals, fill_value="extrapolate")

# --- Define ODE system ---
def pendulum_dynamics(t, y):
    theta, theta_dot = y
    theta_ddot = (tau(t) - b*theta_dot - k*theta) / J
    return [theta_dot, theta_ddot]

# --- Solve the model ODE ---
y0 = [pos_measured[0], 0.0]  # start from measured initial position
sol = solve_ivp(pendulum_dynamics, [t_vals[0], t_vals[-1]], y0, t_eval=t_vals)

theta_sim = sol.y[0]

# --- Compare simulated vs measured ---
plt.figure(figsize=(8,5))
plt.plot(t_vals, pos_measured, label="Measured θ(t)", color='green')
plt.plot(t_vals, theta_sim, label="Simulated θ(t)", color='red', linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Time-Domain Model Validation")
plt.legend()
plt.grid(True)
plt.show()

# --- Optional error metric ---
rms_error = np.sqrt(np.mean((pos_measured - theta_sim)**2))
print(f"RMS position error: {rms_error:.4f} rad")
