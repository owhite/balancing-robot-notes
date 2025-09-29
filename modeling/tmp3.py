import pybullet as p
import pybullet_data
import time, math
import cv2
import numpy as np

# -------------------
# Setup PyBullet
# -------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

pendulum_id = p.loadURDF("pendulum.urdf", useFixedBase=True)

num_joints = p.getNumJoints(pendulum_id)
print("Number of joints:", num_joints)

for j in range(num_joints):
    ji = p.getJointInfo(pendulum_id, j)
    print("Joint", j, ":", ji[1].decode(), "axis:", ji[13])

# Use last joint info to get hinge
hinge_origin = ji[14]
hinge_axis   = ji[13]

axis_len = 0.2
axis_endpoint = [
    hinge_origin[0] + hinge_axis[0]*axis_len,
    hinge_origin[1] + hinge_axis[1]*axis_len,
    hinge_origin[2] + hinge_axis[2]*axis_len
]
p.addUserDebugLine(hinge_origin, axis_endpoint, [0,0,1], 3)

# Disable default motors
for j in range(num_joints):
    p.setJointMotorControl2(pendulum_id, j,
                            controlMode=p.VELOCITY_CONTROL,
                            force=0)

p.setGravity(0,0,-9.81)

# Motion parameters
A = math.radians(10)   # amplitude
freq = 0.5             # Hz
omega = 2*math.pi*freq
Kp = 5.0
Kd = 0.5
dt = 1./240.

# -------------------
# Video Writer Setup
# -------------------
width, height = 640, 480
fps = 30
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter("pendulum_sim.mp4", fourcc, fps, (width, height))

# Camera parameters
cam_target = [0, 0, 0]
cam_distance = 1.5
cam_yaw = 50
cam_pitch = -30

# -------------------
# Simulation loop
# -------------------
for step in range(4000):
    t = step * dt
    theta_des = A * math.sin(omega*t)
    theta_dot_des = A * omega * math.cos(omega*t)
    theta, theta_dot = p.getJointState(pendulum_id, 0)[0:2]

    error = theta_des - theta
    derror = theta_dot_des - theta_dot
    torque = Kp*error + Kd*derror

    p.setJointMotorControl2(pendulum_id, 0,
                            controlMode=p.TORQUE_CONTROL,
                            force=torque)

    p.stepSimulation()

    # Only record ~30 FPS (skip frames)
    if step % 8 == 0:

        img = p.getCameraImage(width, height,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL,
                               viewMatrix=p.computeViewMatrixFromYawPitchRoll(
                                   cam_target, cam_distance,
                                   cam_yaw, cam_pitch, 0, 2),
                               projectionMatrix=p.computeProjectionMatrixFOV(
                                   fov=60, aspect=float(width)/height,
                                   nearVal=0.1, farVal=100.0))

        # Extract RGB
        rgb = np.reshape(img[2], (height, width, 4)).astype(np.uint8)[:, :, :3]
        rgb = np.ascontiguousarray(rgb)

        # Convert to BGR for OpenCV
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Overlay torque (moved down by +40 px)
        torque_text = f"Torque: {torque:.2f} Nm"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        (tw, th), _ = cv2.getTextSize(torque_text, font, font_scale, thickness)
        torque_org = (width - tw - 10, 70)  # 70 px from top instead of 30
        cv2.putText(frame, torque_text, torque_org, font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

        # Overlay time below torque
        time_text = f"Time: {t*1000:.0f} ms"
        (tw2, th2), _ = cv2.getTextSize(time_text, font, font_scale, thickness)
        time_org = (width - tw2 - 10, 70 + 40)  # another 40 px lower
        cv2.putText(frame, time_text, time_org, font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

        print(step)
        out.write(frame)


# -------------------
# Cleanup
# -------------------
out.release()
p.disconnect()
print("Saved video: pendulum_sim.mp4")
