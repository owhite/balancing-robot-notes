#!/usr/bin/env python3
"""
Verify the physical and control consistency of LQR_bot_data.json.

Checks:
  • Confirms positive masses, inertia, and lever arm.
  • Validates controllability of (A, B).
  • Verifies stability of closed-loop (A - B*K) for both continuous and discrete systems.
  • Reports time constants and eigenvalues for physical interpretation.
  • Displays relative magnitudes of fast vs. slow modes.

Usage:
    ./verify_LQR_bot_data.py path/to/LQR_bot_data.json
"""

import json
import numpy as np
import sys
import math

def controllability_matrix(A, B):
    n = A.shape[0]
    return np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])

def observability_matrix(A, C):
    n = A.shape[0]
    return np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(n)])

def verify_bot_data(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)

    print(f"\n🔍 Verifying balancing robot data from: {json_path}\n")

    # --- Extract parameters ---
    p = data["params"]
    m_b = p["m_body"]
    m_w = p["m_wheel"]
    I_b = p["I_body"]
    l = p["l"]
    r = p["r"]
    b_total = p["b_total"]
    g = p["g"]
    Ts = 1.0 / p["sample_rate_Hz"]

    A = np.array(data["A_cont"], dtype=float)
    B = np.array(data["B_cont"], dtype=float)
    A_d = np.array(data["A_disc"], dtype=float)
    B_d = np.array(data["B_disc"], dtype=float)
    K_cont = np.array(data["K_cont"], dtype=float)
    K_disc = np.array(data["K_disc"], dtype=float)

    print("📏 Dimensional sanity checks:")
    print(f"  Body mass (kg):       {m_b:.4f}")
    print(f"  Wheel mass (kg):      {m_w:.4f}")
    print(f"  Inertia (kg·m²):      {I_b:.6e}")
    print(f"  Lever arm l (m):      {l:.4f}")
    print(f"  Wheel radius r (m):   {r:.4f}")
    print(f"  Damping b (N·m·s/rad):{b_total:.4f}")
    print(f"  Gravity (m/s²):       {g:.3f}\n")

    # --- Sanity ---
    if m_b <= 0 or m_w <= 0 or I_b <= 0:
        print("❌ Invalid physical parameters (negative or zero).")
        sys.exit(1)

    # --- Controllability ---
    Co = controllability_matrix(A, B)
    rank_C = np.linalg.matrix_rank(Co)
    print(f"🧮 Controllability rank: {rank_C}/{A.shape[0]}")
    if rank_C < A.shape[0]:
        print("  ⚠️  System may not be fully controllable.")
    else:
        print("  ✅ System is fully controllable.")

    # --- Observability check ---
    C = np.eye(4)  # assume full state measurement
    Ob = observability_matrix(A, C)
    rank_O = np.linalg.matrix_rank(Ob)
    print(f"  Observability rank:   {rank_O}/{A.shape[0]}")
    if rank_O == A.shape[0]:
        print("  ✅ System is fully observable (full-state feedback assumed).")

    # --- Stability check (continuous) ---
    eig_A = np.linalg.eigvals(A)
    eig_closed_cont = np.linalg.eigvals(A - B @ K_cont)

    print("\n⚙️  Continuous-time dynamics:")
    print(f"  Open-loop eigenvalues: {np.round(eig_A, 5)}")
    print(f"  Closed-loop eigenvalues: {np.round(eig_closed_cont, 5)}")

    if np.all(np.real(eig_closed_cont) < 0):
        print("  ✅ Closed-loop system is stable.")
    else:
        print("  ⚠️  Unstable closed-loop poles detected.")

    # --- Stability check (discrete) ---
    eig_Ad = np.linalg.eigvals(A_d)
    eig_closed_disc = np.linalg.eigvals(A_d - B_d @ K_disc)

    print("\n⚙️  Discrete-time dynamics (Ts = {:.6f} s):".format(Ts))
    print(f"  Open-loop eigenvalues: {np.round(eig_Ad, 6)}")
    print(f"  Closed-loop eigenvalues: {np.round(eig_closed_disc, 6)}")

    if np.all(np.abs(eig_closed_disc) < 1.0):
        print("  ✅ Discrete-time closed-loop is stable.")
    else:
        print("  ⚠️  Discrete closed-loop instability detected.")

    # --- Time constants and speed separation ---
    cont_time_constants = []
    for ev in eig_closed_cont:
        if np.real(ev) < 0:
            cont_time_constants.append(-1.0 / np.real(ev))
    cont_time_constants = np.sort(cont_time_constants)

    if len(cont_time_constants) > 0:
        print("\n⏱️  Characteristic time constants (s):", np.round(cont_time_constants, 3))
        fast_modes = cont_time_constants[:2]
        slow_modes = cont_time_constants[-2:]
        print(f"  → Fast modes: ~{np.mean(fast_modes):.2f} s, Slow modes: ~{np.mean(slow_modes):.1f} s")

    # --- Physical plausibility of l ---
    if l < 0.005 or l > 0.5:
        print(f"\n⚠️  Lever arm (l = {l:.4f} m) seems unrealistic — check axle/COM geometry.")
    else:
        print(f"\n✅ Lever arm l = {l:.4f} m appears physically reasonable.")

    # --- Overall verdict ---
    print("\n✅ Verification summary:")
    if (rank_C == A.shape[0]) and np.all(np.real(eig_closed_cont) < 0):
        print("  ✅ Model and LQR data consistent and stable.")
    else:
        print("  ⚠️  Verify input parameters or re-generate LQR data.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./verify_LQR_bot_data.py path/to/LQR_bot_data.json")
        sys.exit(1)
    verify_bot_data(sys.argv[1])
