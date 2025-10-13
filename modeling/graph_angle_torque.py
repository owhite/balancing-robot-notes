#!/usr/bin/env python3
"""
Headless MuJoCo LQR simulation: angle + torque vs time

Reads physical + controller parameters from pendulum_LQR_data.json,
builds an equivalent single-link pendulum model, and runs LQR control.
"""

import json, numpy as np, mujoco
from mujoco import MjModel, MjData
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 2:
    print("Usage: python plot_lqr_response.py pendulum_LQR_data.json [sample_period_s]")
    sys.exit(1)

json_path = sys.argv[1]

# === Load your data ===
with open(json_path, "r") as f:
    cfg = json.load(f)

m = cfg["mass_kg"]
I = cfg["moment_of_inertia_kg_m2"]
r = cfg["r_m"]
b = cfg["motor_params"]["b_Nm_s_per_rad"]
K = np.array(cfg["K_gain"]).reshape(1, 2)
g = cfg["g_m_per_s2"]

# === Build MuJoCo XML dynamically ===
xml = f"""
<mujoco>
  <option timestep="0.0005" gravity="0 0 -{g}"/>
  <worldbody>
    <body name="pendulum" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="{b}" />
      <geom type="capsule" fromto="0 0 0 0 0 {r*2:.4f}" size="0.01" mass="{m}"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"""

model = MjModel.from_xml_string(xml)
data = MjData(model)

# === Initial condition ===
data.qpos[0] = 0.1  # 0.1 rad initial deflection
data.qvel[0] = 0.0

# === Simulation parameters ===
dt = model.opt.timestep
steps = 3000  # 3 seconds at 1 kHz
theta_hist, torque_hist = [], []

# === LQR Closed-loop simulation ===
for _ in range(steps):
    theta = data.qpos[0]
    omega = data.qvel[0]
    x = np.array([theta, omega])
    tau = float(-K @ x)
    data.ctrl[0] = tau
    mujoco.mj_step(model, data)
    theta_hist.append(theta)
    torque_hist.append(tau)

# === Time vector ===
t = np.arange(steps) * dt

# === Plot results ===
fig, ax = plt.subplots(2, 1, figsize=(7, 6), sharex=True)

# Angle plot
ax[0].plot(t, theta_hist, color='tab:blue', label="Angle θ [rad]")
ax[0].set_ylabel("Angle θ [rad]")
ax[0].grid(True)
ax[0].legend()
ax[0].set_title("LQR Pendulum Response (MuJoCo, Headless)")

# Torque plot
ax[1].plot(t, torque_hist, color='tab:orange', label="Torque τ [N·m]")
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Torque [N·m]")
ax[1].grid(True)
ax[1].legend()

plt.tight_layout()
plt.show()
