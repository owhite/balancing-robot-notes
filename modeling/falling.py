import pybullet as p
import pybullet_data
import time

# Connect to PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Load pendulum
pendulum_id = p.loadURDF("pendulum.urdf", useFixedBase=True)

# Print joint info
num_joints = p.getNumJoints(pendulum_id)
print("Number of joints:", num_joints)
for j in range(num_joints):
    print("Joint", j, ":", p.getJointInfo(pendulum_id, j))

# Print link dynamics
print("\nDynamics Info:")
print("Base link (-1):", p.getDynamicsInfo(pendulum_id, -1))
print("Pendulum link (0):", p.getDynamicsInfo(pendulum_id, 0))

# Disable all default joint motors so gravity can act
for j in range(num_joints):
    p.setJointMotorControl2(pendulum_id, j,
                            controlMode=p.VELOCITY_CONTROL,
                            force=0)

# Reduce damping (so it falls freely)
p.changeDynamics(pendulum_id, -1, linearDamping=0, angularDamping=0)
p.changeDynamics(pendulum_id, 0, linearDamping=0, angularDamping=0)

# Set gravity
p.setGravity(0, 0, -9.81)

# Debugging: COM and joint
mass, _, inertia_diag, com_pos, _, *_ = p.getDynamicsInfo(pendulum_id, 0)
print("\nLink 0 COM (local frame):", com_pos, "mass:", mass)
print("Inertia diagonal:", inertia_diag)

# Draw COM marker
p.addUserDebugLine(com_pos, [com_pos[0], com_pos[1], com_pos[2] + 0.02], [1,0,0], 3)

# Get joint info
ji = p.getJointInfo(pendulum_id, 0)
hinge_origin = ji[14]   # parentFramePos
hinge_axis   = ji[13]   # axis vector

print("Joint parentFramePos:", hinge_origin, "axis:", hinge_axis)

# Define box 1 (collision + visual)
collision_shape1 = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.005, 0.025, 0.005]
)
visual_shape1 = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.005, 0.025, 0.005],
    rgbaColor=[1, 0, 0, 1]  # red
)

# Define box 2 (collision + visual)
collision_shape2 = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.005, 0.025, 0.005]
)
visual_shape2 = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.005, 0.025, 0.005],
    rgbaColor=[1, 0, 0, 1]  # red
)

# Create two independent static boxes
box1_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=collision_shape1,
    baseVisualShapeIndex=visual_shape1,
    basePosition=[0.075718, 0.020224, 0.161480]  # meters
)

box2_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=collision_shape2,
    baseVisualShapeIndex=visual_shape2,
    basePosition=[-0.075718, 0.020224, 0.161480]  # meters
)

# Draw hinge axis (blue line)
axis_endpoint = [hinge_origin[0] + hinge_axis[0]*0.2,
                 hinge_origin[1] + hinge_axis[1]*0.2,
                 hinge_origin[2] + hinge_axis[2]*0.2]
p.addUserDebugLine(hinge_origin, axis_endpoint, [0,0,1], 2)

print("----------- Simulation starting -----------")

# Let gravity make it fall
for step in range(2000):
    if step % 120 == 0:  # print every half second
        joint_state = p.getJointState(pendulum_id, 0)
        print(f"Step {step}: angle={joint_state[0]:.3f} rad, vel={joint_state[1]:.3f} rad/s, torque={joint_state[3]}")
    p.stepSimulation()
    time.sleep(1./240.)  # 240 Hz default

p.disconnect()
