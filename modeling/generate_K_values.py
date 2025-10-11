#!/usr/bin/env python3
import json
import numpy as np
from scipy.linalg import solve_continuous_are

# Load the pendulum parameters
with open("pendulum_LQR_data.json", "r") as f:
    data = json.load(f)

# Extract dynamics matrices
A = np.array(data["A_matrix"])
B = np.array(data["B_matrix"])

# Choose LQR weights
Q = np.diag([10.0, 1.0])   # penalize angle and angular velocity
R = np.array([[1.0]])      # penalize torque effort

# Solve the continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)

# Compute the optimal feedback gain K
K = np.linalg.inv(R) @ B.T @ P

# Closed-loop system
Acl = A - B @ K
eigs = np.linalg.eigvals(Acl)

print("✅ LQR gain K =", K)
print("Closed-loop eigenvalues:", eigs)
