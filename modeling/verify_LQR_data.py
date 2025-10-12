#!/usr/bin/env python3
"""
Verify the physical and control consistency of pendulum_LQR_data.json.

Enhancements:
  • Detects inclusion of motor damping term (b/I)
  • Distinguishes expected ΔA from true inconsistency
  • Reports damping magnitude if present

Usage:
    ./verify_pendulum_LQR_data.py path/to/pendulum_LQR_data.json
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

def verify_pendulum_data(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)

    print(f"\n🔍 Verifying pendulum data from: {json_path}\n")

    # Extract values
    m = data["mass_kg"]
    I = data["moment_of_inertia_kg_m2"]
    r = data["r_m"]
    g = data["g_m_per_s2"]
    mgr_over_I_stored = data["mgr_over_I"]
    omega_n_stored = data["omega_n_rad_per_s"]
    period_stored = data["expected_period_s"]
    A_stored = np.array(data["A_matrix"], dtype=float)
    B_stored = np.array(data["B_matrix"], dtype=float)
    motor_params = data.get("motor_params", {})

    # --- Dimensional sanity checks ---
    print("📏 Dimensional sanity checks:")
    print(f"  Mass (kg): {m:.6f}")
    print(f"  Moment of inertia (kg·m²): {I:.6e}")
    print(f"  Lever arm (m): {r:.6f}")
    print(f"  Gravity (m/s²): {g:.2f}")

    # --- Derived quantities ---
    mgr_over_I_calc = (m * g * r) / I
    omega_n_calc = math.sqrt(mgr_over_I_calc)
    period_calc = 2 * math.pi / omega_n_calc

    # --- Consistency checks ---
    print("\n📊 Consistency checks:")
    print(f"  mgr/I stored = {mgr_over_I_stored:.6f}")
    print(f"  mgr/I calc   = {mgr_over_I_calc:.6f}")
    print(f"  Δ mgr/I (%)  = {100 * (mgr_over_I_calc - mgr_over_I_stored) / mgr_over_I_stored:+.3f}%")

    print(f"\n  ω_n stored   = {omega_n_stored:.6f} rad/s")
    print(f"  ω_n calc     = {omega_n_calc:.6f} rad/s")
    print(f"  Δ ω_n (%)    = {100 * (omega_n_calc - omega_n_stored) / omega_n_stored:+.3f}%")

    print(f"\n  T stored     = {period_stored:.6f} s")
    print(f"  T calc       = {period_calc:.6f} s")
    print(f"  Δ T (%)      = {100 * (period_calc - period_stored) / period_stored:+.3f}%")

    # --- Matrix verification ---
    print("\n🧮 Matrix verification:")
    A_expected = np.array([[0, 1], [mgr_over_I_calc, 0]])
    B_expected = np.array([[0], [1 / I]])

    A_diff = np.linalg.norm(A_stored - A_expected)
    B_diff = np.linalg.norm(B_stored - B_expected)
    print(f"  ||ΔA||₂ = {A_diff:.3e}")
    print(f"  ||ΔB||₂ = {B_diff:.3e}")

    # Detect damping term
    b_term_est = -A_stored[1, 1] * I
    has_damping = abs(A_stored[1, 1]) > 1e-6

    if has_damping:
        print(f"\n⚙️  Detected damping term in A[1,1]: {A_stored[1,1]:+.3f} s⁻¹")
        print(f"  → Implies effective b ≈ {b_term_est:.5f} N·m·s/rad")

        if "motor_params" in data and "b_Nm_s_per_rad" in motor_params:
            b_expected = motor_params["b_Nm_s_per_rad"]
            diff_pct = 100 * (b_term_est - b_expected) / b_expected
            print(f"  Motor model b = {b_expected:.5f} → Δ = {diff_pct:+.2f}%")
            if abs(diff_pct) < 10:
                print("  ✅ Damping term matches motor electrical model.")
            else:
                print("  ⚠️  Damping term differs significantly from motor estimate.")
        else:
            print("  ⚠️  No motor_params found; cannot verify b term source.")
    else:
        print("\nℹ️  No damping term detected (A[1,1] ≈ 0). Using undamped model.")

    # --- Stability and controllability ---
    print("\n⚙️  Control system analysis:")
    eigvals = np.linalg.eigvals(A_stored)
    eig_str = ", ".join([f"{ev.real:+.4f}{ev.imag:+.4f}j" for ev in eigvals])
    print(f"  Eigenvalues of A: [{eig_str}]")

    if np.any(np.real(eigvals) > 0):
        print("  ⚠️  System is unstable (expected for inverted pendulum).")
    else:
        print("  ✅ System is stable (normal pendulum orientation).")

    C = np.array([[1, 0]])
    Co = controllability_matrix(A_stored, B_stored)
    Ob = observability_matrix(A_stored, C)

    rank_C = np.linalg.matrix_rank(Co)
    rank_O = np.linalg.matrix_rank(Ob)
    n = A_stored.shape[0]

    print(f"  Controllability rank: {rank_C}/{n}")
    print(f"  Observability rank:   {rank_O}/{n}")

    # --- Verdict ---
    print("\n✅ Verification summary:")
    if (
        abs((mgr_over_I_calc - mgr_over_I_stored) / mgr_over_I_stored) < 1e-3 and
        abs((omega_n_calc - omega_n_stored) / omega_n_stored) < 1e-3 and
        B_diff < 1e-6 and
        rank_C == n and rank_O == n
    ):
        if has_damping:
            print("  ✅ Model verified with damping included; all checks consistent.")
        else:
            print("  ✅ Model verified (no damping term).")
    else:
        print("  ⚠️  One or more checks failed — verify JSON generation or model parameters.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./verify_pendulum_LQR_data.py path/to/pendulum_LQR_data.json")
        sys.exit(1)
    verify_pendulum_data(sys.argv[1])
