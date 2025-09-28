import pybullet as p
import pybullet_data
import time, math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Pendulum dimensions
length, width, thickness = 0.35, 0.06, 0.0053
mass = 0.075

half_extents = [width/2, thickness/2, length/2]

# Shift box geometry upward so base is at hinge
col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents,
                                collisionFramePosition=[0,0,length/2])
vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                             visualFramePosition=[0,0,length/2],
                             rgbaColor=[0.2,0.6,1,1])

# Hinge at z = 0.05
base_pos = [0,0,0.05]

pendulum_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=-1,
    basePosition=base_pos,
    linkMasses=[mass],
    linkCollisionShapeIndices=[col_id],
    linkVisualShapeIndices=[vis_id],
    linkPositions=[[0,0,0]],
    linkOrientations=[[0,0,0,1]],
    linkInertialFramePositions=[[0,0,length/2]],
    linkInertialFrameOrientations=[[0,0,0,1]],
    linkParentIndices=[0],
    linkJointTypes=[p.JOINT_REVOLUTE],
    linkJointAxis=[[0,1,0]]   # rotate about y
)

# Disable default motor
p.setJointMotorControl2(pendulum_id, 0, controlMode=p.VELOCITY_CONTROL, force=0)
p.setGravity(0,0,-9.81)

# Desired motion
A = math.radians(10)   # 10 degrees amplitude
freq = 0.5             # Hz
omega = 2*math.pi*freq

# PD gains
Kp = 5.0
Kd = 0.5

dt = 1./240.
for step in range(4000):
    t = step * dt

    # Desired trajectory
    theta_des = A * math.sin(omega*t)
    theta_dot_des = A * omega * math.cos(omega*t)

    # Current state
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

p.disconnect()
