#!/usr/bin/env python3
"""
simulate_pendulum.py

Step 4: Simulate the fitted pendulum model using a sinusoidal torque input.

Equation of motion:
    J * theta_ddot + b * theta_dot + k * theta = tau(t)

where:
    tau(t) = amp_torque * sin(2π * freq_hz * t)

The model parameters (J, b, k) are loaded from pendulum_model.json,
which should have keys:
    { "J": ..., "b": ..., "k": ... }

This script produces plots of torque, angular position, and velocity vs. time.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# ----------------------------------------------------------------------
# --- Load model parameters ---
# ----------------------------------------------------------------------
with open("pendulum_model.json", "r") as f:
    model = json.load(f)

J = model["J"]   # rotational inertia (kg·m²)
b = model["b"]   # viscous damping coefficient (N·m·s/rad)
k = model["k"]   # stiffness or m*g*l term (N·m/rad)

print("Loaded model parameters:")
print(f"  J = {J:.6f} kg·m²")
print(f"  b = {b:.6f} N·m·s/rad")
print(f"  k = {k:.6f} N·m/rad")
print()

# ----------------------------------------------------------------------
# --- Simulation parameters (user-editable) ---
# ----------------------------------------------------------------------
amp_torque = 0.1     # Nm, amplitude of torque input
freq_hz = 1.0         # Hz, frequency of torque input
duration_s = 5.0      # seconds, total simulation time
dt = 0.002            # seconds, time step for output (2 ms)

# ----------------------------------------------------------------------
# --- Define the system dynamics ---
# ----------------------------------------------------------------------
def pendulum_dynamics(t, y):
    """
    y[0] = theta (rad)
    y[1] = theta_dot (rad/s)
    """
    theta, theta_dot = y
    tau = amp_torque * np.sin(2 * np.pi * freq_hz * t)
    theta_ddot = (tau - b * theta_dot - k * theta) / J
    return [theta_dot, theta_ddot]

# Initial conditions (pendulum hanging at rest)
y0 = [0.0, 0.0]

# ----------------------------------------------------------------------
# --- Integrate the ODE ---
# ----------------------------------------------------------------------
t_span = (0, duration_s)
t_eval = np.arange(0, duration_s, dt)

sol = solve_ivp(pendulum_dynamics, t_span, y0, t_eval=t_eval, method="RK45")

t = sol.t
theta = sol.y[0]
theta_dot = sol.y[1]
torque = amp_torque * np.sin(2 * np.pi * freq_hz * t)

# ----------------------------------------------------------------------
# --- Plot results ---
# ----------------------------------------------------------------------
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
plt.subplots_adjust(hspace=0.3)

ax1.plot(t * 1000, torque, color="tab:red", label="Torque input (Nm)")
ax1.set_ylabel("Torque (Nm)")
ax1.set_title("Sinusoidal Torque Input vs Time")
ax1.grid(True)
ax1.legend()

ax2.plot(t * 1000, theta, color="tab:green", label="Position (rad)")
ax2.set_ylabel("Angle (rad)")
ax2.set_title("Pendulum Position vs Time")
ax2.grid(True)
ax2.legend()

ax3.plot(t * 1000, theta_dot, color="tab:blue", label="Velocity (rad/s)")
ax3.set_xlabel("Time (ms)")
ax3.set_ylabel("Angular velocity (rad/s)")
ax3.set_title("Pendulum Velocity vs Time")
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.show()
