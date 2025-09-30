#!/usr/bin/env python3
# Compare pendulum swing under 0.2 Nm vs 0.4 Nm torque
import pybullet as p
import pybullet_data
import math
import matplotlib.pyplot as plt

def run_sim(torque_mag, sim_time=1.0):
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    pendulum_id = p.loadURDF("pendulum.urdf", useFixedBase=True)

    # disable default motor
    p.setJointMotorControl2(pendulum_id, 0,
                            controlMode=p.VELOCITY_CONTROL, force=0)

    dt = 1.0/240.0
    steps = int(sim_time / dt)
    t_data, theta_data = [], []

    reached = False
    for step in range(steps):
        t = step * dt
        theta, theta_dot = p.getJointState(pendulum_id, 0)[0:2]

        if not reached:
            if theta < math.pi/2:
                torque = torque_mag
            else:
                reached = True
                torque = 0.0
        else:
            torque = 0.0

        p.setJointMotorControl2(pendulum_id, 0,
                                controlMode=p.TORQUE_CONTROL,
                                force=torque)
        p.stepSimulation()

        if step % 10 == 0:  # log at ~24 Hz
            t_data.append(t)
            theta_data.append(theta)

    return t_data, theta_data

# Connect PyBullet in DIRECT mode (no GUI)
p.connect(p.DIRECT)

# Run 0.2 Nm and 0.4 Nm
t1, th1 = run_sim(0.2)
t2, th2 = run_sim(0.4)

p.disconnect()

# Plot
plt.figure()
plt.plot(t1, th1, label="0.2 Nm")
plt.plot(t2, th2, label="0.4 Nm")
plt.axhline(math.pi/2, color='gray', linestyle='--', label="π/2 rad")
plt.xlabel("Time (s)")
plt.ylabel("Angle θ (rad)")
plt.legend()
plt.title("Pendulum response: 0.2 Nm vs 0.4 Nm")
plt.grid(True)
plt.show()
