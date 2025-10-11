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
p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
p.setGravity(0, 0, -9.81)

# -------------------------------
# Load environment and model
# -------------------------------
p.loadURDF("plane.urdf")
pendulum_id = p.loadURDF(URDF_PATH, basePosition=[0, 0, 0], useFixedBase=True)

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
try:
    with open(JSON_PATH, "r") as f:
        meta = json.load(f)
    joint_origin_from_json = np.array(meta.get("origin", [0, 0, 0])) * 0.001  # mm → m
    joint_axis_from_json = np.array(meta.get("axis_of_rotation", [0, 1, 0]))
    joint_axis_from_json = joint_axis_from_json / np.linalg.norm(joint_axis_from_json)
    print(f"\n✅ JSON joint origin (m): {joint_origin_from_json}")
    print(f"✅ JSON joint axis: {joint_axis_from_json}")
except Exception as e:
    print("\n⚠️ Could not read metadata file, using defaults:", e)
    joint_origin_from_json = np.array([0, 0, 0.4])
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
# Simulation loop
# -------------------------------
print("\nRunning simulation — close the window to exit.")
while p.isConnected():
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
