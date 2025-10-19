#!/usr/bin/env python3
"""
Recompute discrete-time LQR gain (K_gain_discrete) using parameters from a param file.

Usage:
    ./recalculate_K_gain_discrete.py <param_file.json>

Example:
    ./recalculate_K_gain_discrete.py pendulum_params.json
"""

import json
import sys
import numpy as np
from scipy.linalg import solve_discrete_are


def compute_discrete_lqr_gain(A_d, B_d, Q_term, R_term, B_term):
    """
    Compute discrete-time LQR gain matrix K for:
        x[k+1] = A_d x[k] + B_d u[k]
    using the discrete algebraic Riccati equation (DARE).

    Parameters:
        A_d (ndarray): Discrete-time state matrix
        B_d (ndarray): Discrete-time input matrix
        Q_term (float): Weight on position error (in Q = diag([Q_term, 1.0]))
        R_term (float): Weight on control effort (in R = [[R_term]])
        B_term (float): Input gain factor (included for completeness)

    Returns:
        K (ndarray): 1x2 LQR gain matrix
    """
    Q = np.diag([Q_term, 1.0])
    R = np.array([[R_term]])

    # Solve discrete Riccati equation
    P = solve_discrete_are(A_d, B_d, Q, R)

    # Compute discrete-time LQR gain
    K = np.linalg.inv(B_d.T @ P @ B_d + R) @ (B_d.T @ P @ A_d)
    return K


def main(param_file):
    # --- Load parameter file ---
    with open(param_file, "r") as f:
        params = json.load(f)

    LQR_data_path = params["LQR_data"]
    Q_term = params["Qterm"]
    R_term = params["Rterm"]
    B_term = params["Bterm"]

    # --- Load LQR system data ---
    with open(LQR_data_path, "r") as f:
        data = json.load(f)

    A_d = np.array(data["A_matrix_discrete"])
    B_d = np.array(data["B_matrix_discrete"])

    # --- Compute discrete LQR gain ---
    K_disc = compute_discrete_lqr_gain(A_d, B_d, Q_term, R_term, B_term)

    print("\n=== DISCRETE-TIME LQR GAIN RECOMPUTATION ===")
    print(f"Parameter file: {param_file}")
    print(f"LQR data file:  {LQR_data_path}")
    print("A_d =\n", A_d)
    print("B_d =\n", B_d)
    print(f"Qterm = {Q_term},  Rterm = {R_term},  Bterm = {B_term}")
    print("K_gain_discrete =", K_disc)

    # --- Add recomputed gain to JSON ---
    data["K_gain_discrete_recomputed"] = K_disc.tolist()



if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./recalculate_K_gain_discrete.py <param_file.json>")
        sys.exit(1)
    main(sys.argv[1])
