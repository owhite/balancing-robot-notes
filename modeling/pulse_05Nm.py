#!/usr/bin/env python3
# Pendulum: swing to π/2 rad with +0.5 Nm torque, then release
# Overlays torque, time, and torque-on duration

import pybullet as p
import pybullet_data
import math
import cv2
import numpy as np

# -------------------
# Setup PyBullet
# -------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

pendulum_id = p.loadURDF("pendulum.urdf", useFixedBase=True)
p.setGravity(0, 0, -9.81)

# Disable default motors
num_joints = p.getNumJoints(pendulum_id)
for j in range(num_joints):
    p.setJointMotorControl2(pendulum_id, j,
                            controlMode=p.VELOCITY_CONTROL,
                            force=0)

theta0 = p.getJointState(pendulum_id, 0)[0]
print("Initial joint angle (rad):", theta0)

# -------------------
# Motion parameters
# -------------------
target_angle = math.pi / 2   # horizontal
torque_mag = 0.2             # Nm
dt = 1./240.

# -------------------
# Video Writer Setup
# -------------------
width, height = 640, 480
fps = 30
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter("pendulum_torque_05Nm.mp4", fourcc, fps, (width, height))

cam_target = [0, 0, 0]
cam_distance = 1.5
cam_yaw = 50
cam_pitch = -30

# -------------------
# Simulation loop
# -------------------
reached = False
torque_on_time = 0.0  # ms counter for how long torque has been applied
total_steps = 4000    # ~10 seconds

for step in range(total_steps):
    t = step * dt
    theta, theta_dot = p.getJointState(pendulum_id, 0)[0:2]

    # Phase 1: apply torque until π/2
    if not reached:
        if theta < target_angle:
            torque = torque_mag
            torque_on_time += dt * 1000.0  # accumulate in ms
        else:
            reached = True
            torque = 0.0
        font_color = (0, 255, 255)
    else:
        torque = 0.0
        font_color = (255, 255, 255)

    p.setJointMotorControl2(pendulum_id, 0,
                            controlMode=p.TORQUE_CONTROL,
                            force=torque)
    p.stepSimulation()

    # Record ~30 FPS
    if step % 8 == 0:
        img = p.getCameraImage(width, height,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL,
                               viewMatrix=p.computeViewMatrixFromYawPitchRoll(
                                   cam_target, cam_distance,
                                   cam_yaw, cam_pitch, 0, 2),
                               projectionMatrix=p.computeProjectionMatrixFOV(
                                   fov=60, aspect=float(width)/height,
                                   nearVal=0.1, farVal=100.0))

        rgb = np.reshape(img[2], (height, width, 4)).astype(np.uint8)[:, :, :3]
        rgb = np.ascontiguousarray(rgb)
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Overlay torque info
        if torque != 0.0:
            torque_text = f"Torque: {torque:.2f} Nm (APPLIED)"
        else:
            torque_text = f"Torque: {torque:.2f} Nm (OFF)"

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2

        cv2.putText(frame, torque_text,
                    (20, 40), font, font_scale, (255, 255, 255),
                    thickness, cv2.LINE_AA)

        # Overlay time
        time_text = f"Time: {t*1000:.0f} ms"
        cv2.putText(frame, time_text,
                    (20, 80), font, font_scale, (255, 255, 255),
                    thickness, cv2.LINE_AA)

        if (torque > .00001): 
            # Overlay torque-on duration
            duration_text = f"Torque duration: {torque_on_time:.0f} ms"
            cv2.putText(frame, duration_text,
                        (20, 120), font, font_scale, font_color,
                        thickness, cv2.LINE_AA)

        out.write(frame)
        print(f"step={step}, theta={theta:.2f}, torque={torque:.2f}, applied={torque_on_time:.1f} ms")

# -------------------
# Cleanup
# -------------------
out.release()
p.disconnect()
print("Saved video: pendulum_torque_05Nm.mp4")
