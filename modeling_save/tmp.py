#!/usr/bin/env python3

import numpy as np
from scipy.linalg import expm, solve_discrete_are

Ts = 0.002  # 500 Hz

A = np.array([[0, 1],
              [38.79, -27.02]])
B = np.array([[0],
              [102.0]])
Q = np.diag([10, 1])
R = np.array([[1]])

# Discretize (exact ZOH)
M = np.block([[A, B],
              [np.zeros((1, 3))]])
Md = expm(M * Ts)
A_d = Md[:2, :2]
B_d = Md[:2, 2:3]

# Solve discrete-time Riccati equation
P = solve_discrete_are(A_d, B_d, Q, R)

# Compute discrete LQR gain
K = np.linalg.inv(B_d.T @ P @ B_d + R) @ (B_d.T @ P @ A_d)

# Closed-loop poles
eigvals = np.linalg.eigvals(A_d - B_d @ K)

print("A_d =\n", A_d)
print("B_d =\n", B_d)
print("K =", np.round(K, 3))
print("Closed-loop poles =", np.round(eigvals, 3))
