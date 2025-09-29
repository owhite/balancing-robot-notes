import pybullet as p
import pybullet_data
import time, math

# Connect
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Start video logging (MP4)
log_id = p.startStateLogging(
    p.STATE_LOGGING_VIDEO_MP4,
    "pendulum_sim.mp4"
)

# Load your URDF
pendulum_id = p.loadURDF("pendulum.urdf", useFixedBase=True)

# Print info
num_joints = p.getNumJoints(pendulum_id)
print("Number of joints:", num_joints)

for j in range(num_joints):
    ji = p.getJointInfo(pendulum_id, j)
    print("Joint", j, ":", ji[1].decode(), "axis:", ji[13])

# Hinge joint origin and axis from URDF
hinge_origin = ji[14]   # parentFramePos (relative to base_link)
hinge_axis   = ji[13]   # axis (local)

# Draw hinge axis line (blue)
axis_len = 0.2  # meters
axis_endpoint = [
    hinge_origin[0] + hinge_axis[0]*axis_len,
    hinge_origin[1] + hinge_axis[1]*axis_len,
    hinge_origin[2] + hinge_axis[2]*axis_len
]
p.addUserDebugLine(hinge_origin, axis_endpoint, [0,0,1], 3)

# Disable default motor so we can use torque control
for j in range(num_joints):
    p.setJointMotorControl2(pendulum_id, j,
                            controlMode=p.VELOCITY_CONTROL,
                            force=0)

# Gravity
p.setGravity(0,0,-9.81)

# Desired motion: sinusoidal angle
A = math.radians(10)   # ±10 degrees
freq = 0.5             # Hz
omega = 2*math.pi*freq

# PD gains
Kp = 5.0
Kd = 0.5

dt = 1./240.
for step in range(400):
    t = step * dt

    # Desired trajectory
    theta_des = A * math.sin(omega*t)
    theta_dot_des = A * omega * math.cos(omega*t)

    # Current joint state
    theta, theta_dot = p.getJointState(pendulum_id, 0)[0:2]

    # PD control torque
    error = theta_des - theta
    derror = theta_dot_des - theta_dot
    torque = Kp*error + Kd*derror

    # Apply torque
    p.setJointMotorControl2(pendulum_id, 0,
                            controlMode=p.TORQUE_CONTROL,
                            force=torque)

    p.stepSimulation()
    time.sleep(dt)

p.stopStateLogging(log_id)
p.disconnect()
