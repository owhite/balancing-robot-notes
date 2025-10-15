#!/usr/bin/env python3
import numpy as np
from scipy.linalg import solve_continuous_are

A = np.array([[0, 1],
              [38.79, -27]])       # no damping
B = np.array([[0],
              [102]])            # B_real
Q = np.diag([50, 1])
R = np.array([[.1]])            # smaller control cost → higher gain

P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ (B.T @ P)
eigvals = np.linalg.eigvals(A - B @ K)

print("K =", np.round(K, 2))
print("Closed-loop poles =", np.round(eigvals, 2))
