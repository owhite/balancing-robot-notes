#!/usr/bin/env mjpython
"""
MuJoCo LQR pendulum visualization (with periodic perturbations).
Shows a small yellow capsule at the hinge location.

Usage:
    mjpython visualize_pendulum.py pendulum_LQR_data.json
"""

import json, sys, time, numpy as np, mujoco
from mujoco import MjModel, MjData
import mujoco.viewer

# === Check arguments ===
if len(sys.argv) < 2:
    print("Usage: mjpython visualize_pendulum.py <pendulum_LQR_data.json>")
    sys.exit(1)

json_path = sys.argv[1]

# === Load parameters ===
with open(json_path, "r") as f:
    cfg = json.load(f)

m = cfg["mass_kg"]
r = cfg["r_m"]
b = cfg["motor_params"]["b_Nm_s_per_rad"]
K = np.array(cfg["K_gain"]).reshape(1, 2)
g = cfg["g_m_per_s2"]

# === Build MuJoCo model ===
xml = f"""
<mujoco>
  <option timestep="0.0005" gravity="0 0 -{g}"/>
  <worldbody>
    <!-- Static base for visual reference -->
    <body name="base" pos="0 0 0">
      <!-- Small yellow capsule marking hinge location -->
      <geom type="capsule"
       fromto="0 -0.05 0  0 0.05 0"
       size="0.008"
       rgba="1 1 0 1"
       contype="0" conaffinity="0"/>
    </body>

    <!-- Pendulum body -->
    <body name="pendulum" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="{b}"/>
      <geom type="capsule" fromto="0 0 0 0 0 {r*2:.4f}"
            size="0.015" mass="{m}" rgba="0.4 0.6 1 1"/>
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
data.qpos[0] = 0.05   # small deflection
data.qvel[0] = 0.0

# === Simulation config ===
perturb_interval = 3.0      # seconds between perturbations
perturb_torque = 0.5        # N·m impulse
perturb_duration = 0.05     # seconds
last_perturb = 0.0
dt = model.opt.timestep

print(f"✅ Viewer starting — loaded {json_path}")
print("Close window to exit.\n")

# === Viewer setup ===
viewer = mujoco.viewer.launch_passive(model, data)

# Preferred camera view
viewer.cam.azimuth = 154
viewer.cam.elevation = -44.0
viewer.cam.distance = 0.5
viewer.cam.lookat[:] = [-0.08610948, 0.02539914, -0.01537349]

# === Continuous loop ===
while viewer.is_running():
    theta, omega = data.qpos[0], data.qvel[0]
    tau = float(-K @ np.array([theta, omega]))

    # Apply perturbation periodically
    sim_t = data.time
    if sim_t - last_perturb > perturb_interval:
        print(f"⚡ Perturbation at t={sim_t:.2f}s")
        for _ in range(int(perturb_duration / dt)):
            data.ctrl[0] = tau + perturb_torque
            mujoco.mj_step(model, data)
        last_perturb = sim_t

    # Normal control step
    data.ctrl[0] = tau
    mujoco.mj_step(model, data)

    # Sync viewer at roughly real time
    viewer.sync()
    time.sleep(dt)

print("Viewer closed — exiting cleanly.")
