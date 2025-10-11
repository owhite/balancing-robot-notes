#!/usr/bin/env python3
import numpy as np
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

pendulum_id = p.loadURDF("/Users/owhite/MESC_brain_board/modeling/pendulum_assembly.urdf", useFixedBase=True)

# find your joint
joint_index = 0  # since you only have one, pendulum_joint

# get joint info
joint_info = p.getJointInfo(pendulum_id, joint_index)
joint_name = joint_info[1].decode("utf-8")

# the joint frame relative to parent (in URDF)
joint_pos_parent = np.array(joint_info[14])  # position in parent link frame
joint_axis_local = np.array(joint_info[13])  # axis in parent link frame

# get base (parent) transform in world frame
base_pos, base_orn = p.getBasePositionAndOrientation(pendulum_id)

# convert joint frame to world coordinates
joint_origin_world, _ = p.multiplyTransforms(
    base_pos, base_orn, joint_pos_parent.tolist(), [0, 0, 0, 1]
)
joint_axis_world = p.rotateVector(base_orn, joint_axis_local)

print("Joint origin (world):", joint_origin_world)
print("Joint axis (world):", joint_axis_world)

# draw the yellow line correctly at the hinge
axis_len = 0.2
start = np.array(joint_origin_world) - np.array(joint_axis_world) * axis_len / 2
end = np.array(joint_origin_world) + np.array(joint_axis_world) * axis_len / 2
p.addUserDebugLine(start.tolist(), end.tolist(), [1, 1, 0], 3)
p.addUserDebugText("joint", joint_origin_world, [1, 1, 0], textSize=1.5)
