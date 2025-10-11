#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import json

# -------------------------------
# Paths
# -------------------------------
URDF_PATH = "/Users/owhite/MESC_brain_board/modeling/pendulum_assembly.urdf"
JSON_PATH = "/Users/owhite/MESC_brain_board/modeling/pendulum_metadata.json"

# -------------------------------
# PyBullet setup
# -------------------------------
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# -------------------------------
# Load environment and model
# -------------------------------
p.loadURDF("plane.urdf")
pendulum_id = p.loadURDF(URDF_PATH, basePosition=[0,0,0], useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

print(f"\n✅ Loaded URDF id = {pendulum_id}")
num_joints = p.getNumJoints(pendulum_id)
print(f"Number of joints: {num_joints}")
for i in range(num_joints):
    info = p.getJointInfo(pendulum_id, i)
    print(f"  {i}: {info[1].decode('utf-8')} (type={info[2]})")

# -------------------------------
# Draw world axes for reference
# -------------------------------
axis_len = 0.1
p.addUserDebugLine([0, 0, 0], [axis_len, 0, 0], [1, 0, 0], 2)  # X red
p.addUserDebugLine([0, 0, 0], [0, axis_len, 0], [0, 1, 0], 2)  # Y green
p.addUserDebugLine([0, 0, 0], [0, 0, axis_len], [0, 0, 1], 2)  # Z blue
p.addUserDebugText("World Origin", [0, 0, 0], [1, 1, 1], textSize=1.4)

# -------------------------------
# Read joint info from URDF and JSON (if available)
# -------------------------------
# --- Load theoretical data from JSON (if available) ---
try:
    with open(JSON_PATH, "r") as f:
        meta = json.load(f)
    print(f"\n✅ Loaded metadata from {JSON_PATH}")

    # Try to get values; fall back gracefully if missing
    mgr_over_I = meta.get("mgr_over_I", None)
    if mgr_over_I is None:
        print("⚠️ 'mgr_over_I' not found in JSON — computing manually.")
        g = 9.81
        m = meta.get("total_mass", 0.193058)
        r = np.linalg.norm(np.array(meta.get("total_com", [0, 0, 89])) - np.array(meta.get("origin_of_rotation", [0, 0, 0]))) * 0.001
        I = meta.get("moment_of_inertia_scalar_m2", 0.004923982)
        mgr_over_I = (m * g * r) / I

    lambda_theory = np.sqrt(mgr_over_I)

    # Joint info from JSON
    joint_origin_from_json = np.array(meta.get("origin_of_rotation", [0, 0, 0])) * 0.001
    joint_axis_from_json = np.array(meta.get("axis_unit_vector", [0, 1, 0]))
    joint_axis_from_json /= np.linalg.norm(joint_axis_from_json)

    print(f"✅ Computed mgr/I = {mgr_over_I:.6f}, λ_theory = {lambda_theory:.3f} 1/s")

except Exception as e:
    print(f"\n⚠️ Could not read metadata file: {e}")
    lambda_theory = 0.0
    joint_origin_from_json = np.array([0, 0, 0.045])
    joint_axis_from_json = np.array([0, 1, 0])



# -------------------------------
# Get PyBullet’s reported joint info
# -------------------------------
if num_joints > 0:
    joint_index = 0  # assuming single joint
    info = p.getJointInfo(pendulum_id, joint_index)

    joint_name = info[1].decode("utf-8")
    joint_axis_local = np.array(info[13])
    joint_origin_local = np.array(info[14])

    base_pos, base_orn = p.getBasePositionAndOrientation(pendulum_id)
    joint_origin_world, _ = p.multiplyTransforms(base_pos, base_orn, joint_origin_local.tolist(), [0, 0, 0, 1])
    joint_axis_world = p.rotateVector(base_orn, joint_axis_local)

    print(f"\n🔍 URDF joint name: {joint_name}")
    print(f"  Local origin: {joint_origin_local}")
    print(f"  Local axis: {joint_axis_local}")
    print(f"  World origin: {joint_origin_world}")
    print(f"  World axis: {joint_axis_world}")

    # -------------------------------
    # Visualize joint origin and axis (PyBullet derived)
    # -------------------------------
    p.addUserDebugText("URDF Joint Origin", joint_origin_world, [1, 0, 0], textSize=1.5)
    axis_len = 0.2
    start = np.array(joint_origin_world) - np.array(joint_axis_world) * axis_len / 2
    end = np.array(joint_origin_world) + np.array(joint_axis_world) * axis_len / 2
    p.addUserDebugLine(start.tolist(), end.tolist(), [1, 1, 0], 3)  # yellow line
    p.loadURDF("sphere_small.urdf", basePosition=joint_origin_world, globalScaling=0.05)

    # -------------------------------
    # Compare to JSON joint (if present)
    # -------------------------------
    delta = np.array(joint_origin_world) - joint_origin_from_json
    print(f"  Δ between URDF and JSON origins: {delta}")

    # Draw JSON axis line (cyan) for comparison
    start_json = joint_origin_from_json - joint_axis_from_json * axis_len / 2
    end_json = joint_origin_from_json + joint_axis_from_json * axis_len / 2
    p.addUserDebugLine(start_json.tolist(), end_json.tolist(), [0, 1, 1], 2)
    p.addUserDebugText("JSON axis", end_json.tolist(), [0, 1, 1], textSize=1.3)

# query the internal inertia tensor for pendulum link
info = p.getDynamicsInfo(pendulum_id, 0)
mass, _, inertia_diag, com_frame_pos, *_ = p.getDynamicsInfo(pendulum_id, 0)
print("\n📊 Bullet-reported inertia frame:")
print(f"  mass = {mass:.6f} kg")
print(f"  local inertia diagonal = {inertia_diag}")
print(f"  inertial frame position = {com_frame_pos}")
r_vec = np.array(com_frame_pos)  # from pivot to COM, in m
r_mag = np.linalg.norm(r_vec)
I_about_com = inertia_diag[1]
I_pivot_calc = I_about_com + mass * r_mag**2
print(f"  ⇒ effective I_pivot = {I_pivot_calc:.6e} kg·m²")

# more testing
info = p.getDynamicsInfo(pendulum_id, 0)
print("📘 Dynamics info (inertial frame position):", info[3])

# Get the local y-axis
mat = np.array(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(pendulum_id)[1])).reshape(3,3)
print("Local axes:", mat)

#Get link state info:
link_state = p.getLinkState(pendulum_id, 0, computeForwardKinematics=False)
print("Bullet link local center of mass (m):", link_state[2])

# -------------------------------
# Visualize link frames
# -------------------------------
for i in range(num_joints):
    link_state = p.getLinkState(pendulum_id, i)
    link_pos = link_state[0]
    link_name = p.getJointInfo(pendulum_id, i)[12].decode("utf-8") or f"link_{i}"
    p.addUserDebugText(f"Link {i}", link_pos, [0, 1, 0], textSize=1.3)
    p.addUserDebugLine(link_pos, np.add(link_pos, [0, 0, 0.05]).tolist(), [0, 1, 0], 2)

# -------------------------------
# Display base position and orientation
# -------------------------------
base_pos, base_orn = p.getBasePositionAndOrientation(pendulum_id)
print(f"\n📦 Base link world position: {base_pos}")
print(f"📦 Base link world orientation (quat): {base_orn}")
p.addUserDebugText("Base Link", base_pos, [1, 0.5, 0], textSize=1.3)

# -------------------------------
# Dynamic validation: will the pendulum topple?
# -------------------------------
print("\n🔬 Running topple test...")

# assume single revolute joint at index 0
joint_index = 0
initial_angle_deg = 5.0       # small tilt
initial_angle_rad = np.deg2rad(initial_angle_deg)

# remove any joint motor control (pure dynamics)
p.setJointMotorControl2(pendulum_id, joint_index,
                        controlMode=p.VELOCITY_CONTROL,
                        force=0)

# set small initial rotation about the joint axis
p.resetJointState(pendulum_id, joint_index, targetValue=initial_angle_rad, targetVelocity=0.0)
print(f"Initial joint angle = {initial_angle_deg:.2f}°")

# -------------------------------
# Dynamic validation: exponential growth measurement
# -------------------------------
print("\n🔬 Measuring topple growth rate...")

joint_index = 0
initial_angle_deg = 5.0
initial_angle_rad = np.deg2rad(initial_angle_deg)

# disable joint motor torque
p.setJointMotorControl2(pendulum_id, joint_index,
                        controlMode=p.VELOCITY_CONTROL,
                        force=0)

# small initial tilt
p.resetJointState(pendulum_id, joint_index,
                  targetValue=initial_angle_rad, targetVelocity=0.0)
print(f"Initial angle: {initial_angle_deg:.2f}°")

link_index = 0  # pendulum link
com_pos, _, _, _, _, _ = p.getLinkState(pendulum_id, link_index)
joint_info = p.getJointInfo(pendulum_id, link_index)
joint_origin_local = np.array(joint_info[14])
base_pos, base_orn = p.getBasePositionAndOrientation(pendulum_id)
joint_origin_world, _ = p.multiplyTransforms(base_pos, base_orn, joint_origin_local.tolist(), [0, 0, 0, 1])

r_vec = np.array(com_pos) - np.array(joint_origin_world)
r_mag = np.linalg.norm(r_vec)
print(f"🔹 CoM world pos: {com_pos}")
print(f"🔹 Joint origin world pos: {joint_origin_world}")
print(f"🔹 Effective lever arm (pivot→CoM): {r_mag:.5f} m")

# simulation parameters
timestep = 1.0 / 240.0
duration_s = 3.0
steps = int(duration_s / timestep)

t_vals, angles = [], []

# --- Verify joint axis alignment ---
axis_declared = np.array([0, 1, 0])
axis_sim = np.array(joint_axis_world) / np.linalg.norm(joint_axis_world)
cosang = abs(np.dot(axis_declared, axis_sim))
angle_deg = np.degrees(np.arccos(cosang))
print(f"🔎 Joint axis deviation from declared [0,1,0]: {angle_deg:.2f}°")

for step in range(steps):
    p.stepSimulation()
    state = p.getJointState(pendulum_id, joint_index)
    t = step * timestep
    angle = state[0]
    t_vals.append(t)
    angles.append(angle)
    time.sleep(timestep)

# convert to numpy arrays
t_vals = np.array(t_vals)
angles = np.array(angles)

# use only early portion (small angles)
mask = np.abs(np.rad2deg(angles)) < 25
t_fit = t_vals[mask]
theta_fit = np.abs(angles[mask])

# avoid zeros
theta_fit = np.clip(theta_fit, 1e-6, None)

# fit ln(|θ|) = λ t + c  → λ = slope
coeffs = np.polyfit(t_fit, np.log(theta_fit), 1)
lambda_sim = coeffs[0]

# theoretical growth rate from JSON values
# mgr_over_I = meta["mgr_over_I"]
mgr_over_I = 34.23932418988005
lambda_theory = np.sqrt(mgr_over_I)

print(f"\n📈 Measured exponential rate λ_sim  = {lambda_sim:6.3f} 1/s")
print(f"📘 Theoretical rate λ_theory       = {lambda_theory:6.3f} 1/s")
print(f"   Ratio λ_sim / λ_theory          = {lambda_sim/lambda_theory:6.3f}")

# interpret result
if 0.9 <= lambda_sim / lambda_theory <= 1.1:
    verdict = "✅ Simulation matches theoretical growth within ~10%"
else:
    verdict = "⚠️ Significant discrepancy; check inertia or pivot alignment"

print(verdict)

# keep GUI open
print("\nClose the window to finish.")
while p.isConnected():
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
