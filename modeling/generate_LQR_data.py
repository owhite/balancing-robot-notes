#!/usr/bin/env python3
"""
Compute the physical parameters, linearized state-space model,
and LQR gain for an inverted pendulum from its CAD-derived JSON configuration.

Usage:
    ./generate_pendulum_LQR_data.py -i <assembly_config.json> -k <Kv> -p <R_phase> -r <R_value> -q <Q_term> -b <B_term>

Example:
    ./generate_LQR_data.py -i pendulum_metadata.json -k 170 -p 0.07 -r 1.0 -q 50 -b 102

This version:
  • Uses per-part masses (grams) from STL metadata, not densities.
  • Builds a 4-state torque-controlled inverted-pendulum-on-wheels model.
  • Reads motor parameters separately (robot_params.json).
  • Outputs LQR_bot_data.json in the same folder as the metadata file.
"""

import json
import numpy as np
import argparse
import os
import trimesh
from scipy.linalg import solve_continuous_are, solve_discrete_are, expm


def calculate_total_inertia_from_mass(assembly_data: dict):
    """Compute total mass, center of mass, and scalar moment of inertia about z-axis (vertical)."""
    total_mass = 0.0
    weighted_com_sum = np.zeros(3)
    total_scalar_inertia = 0.0

    for name, part in assembly_data["meshes"].items():
        mass_g = part["mass"]
        stl_path = os.path.join(assembly_data["file_path"], name)
        mesh = trimesh.load_mesh(stl_path)
        mesh_mass_kg = mass_g * 1e-3

        # Compute inertia tensor about mesh COM, scaled by mass ratio
        I_tensor = mesh.moment_inertia * (mesh_mass_kg / mesh.mass)
        com_mm = mesh.center_mass
        origin = np.zeros(3)

        # Parallel-axis shift (to overall origin)
        r_vec = com_mm - origin
        d = np.linalg.norm(r_vec)
        I_axis = np.trace(I_tensor) / 3.0 + mesh_mass_kg * (d * 1e-3) ** 2  # rough scalar inertia

        total_mass += mesh_mass_kg
        weighted_com_sum += com_mm * mesh_mass_kg
        total_scalar_inertia += I_axis

    total_com = weighted_com_sum / total_mass
    I_total = total_scalar_inertia  # already kg·m²
    return total_mass, total_com, I_total


def compute_lqr_for_balancing_bot(params, m_body, I_body, com_mm, m_wheel):
    """Compute continuous and discrete LQR gains for torque-controlled balancing robot."""
    g = 9.81
    l = np.linalg.norm(com_mm) * 1e-3  # axle-to-CoM distance (m)
    r = params["wheel_radius"] * 1e-3  # wheel radius (m)
    b_total = params["b_Nm_s_per_rad"] + params["friction_term"]

    print("\n=== PHYSICAL PARAMETERS ===")
    print(f"Body mass       m_b = {m_body:.4f} kg")
    print(f"Wheel mass      m_w = {m_wheel:.4f} kg")
    print(f"Moment inertia  I_b = {I_body:.4e} kg·m²")
    print(f"CoM distance     l  = {l:.4f} m")
    print(f"Wheel radius     r  = {r:.4f} m")
    print(f"Total damping     b = {b_total:.4f} N·m·s/rad")

    # --- State-space model ---
    A = np.array([
        [0, 1, 0, 0],
        [(m_body + m_wheel) * g * l / (I_body + m_body * l ** 2), -b_total / (I_body + m_body * l ** 2), 0, 0],
        [0, 0, 0, 1],
        [-m_body * g * l / (m_wheel + m_body), b_total / (m_wheel + m_body), 0, -b_total / (m_wheel + m_body)]
    ])

    B = np.array([
        [0],
        [1 / (I_body + m_body * l ** 2)],
        [0],
        [1 / (m_wheel + m_body)]
    ])

    # LQR weighting (tune as desired)
    Q = np.diag([50, 1, 10, 1])
    R = np.array([[1.0]])

    # Continuous-time LQR
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    eig_cont = np.linalg.eigvals(A - B @ K)

    # Discretize at 500 Hz
    Ts = 0.002
    M = np.block([[A, B], [np.zeros((1, 5))]])
    Md = expm(M * Ts)
    A_d = Md[:4, :4]
    B_d = Md[:4, 4:5]

    P_d = solve_discrete_are(A_d, B_d, Q, R)
    K_d = np.linalg.inv(B_d.T @ P_d @ B_d + R) @ (B_d.T @ P_d @ A_d)
    eig_disc = np.linalg.eigvals(A_d - B_d @ K_d)

    return {
        "A_cont": A.tolist(),
        "B_cont": B.tolist(),
        "A_disc": A_d.tolist(),
        "B_disc": B_d.tolist(),
        "Q": Q.tolist(),
        "R": R.tolist(),
        "K_cont": K.tolist(),
        "K_disc": K_d.tolist(),
        "eig_cont": eig_cont.real.tolist(),
        "eig_disc": eig_disc.real.tolist(),
        "params": {
            "m_body": m_body,
            "m_wheel": m_wheel,
            "I_body": I_body,
            "l": l,
            "r": r,
            "b_total": b_total,
            "g": g,
            "sample_rate_Hz": 1 / Ts
        }
    }


def main():
    parser = argparse.ArgumentParser(description="Compute LQR data for a balancing robot.")
    parser.add_argument("-m", "--metadata", required=True, help="LQR_bot_metadata.json file")
    parser.add_argument("-p", "--params", required=True, help="robot_params.json file")
    args = parser.parse_args()

    with open(args.metadata, "r") as f:
        metadata = json.load(f)
    with open(args.params, "r") as f:
        param_file = json.load(f)
    params = param_file["params"]

    total_mass, com, I_total = calculate_total_inertia_from_mass(metadata)

    # Identify wheel mass
    wheel_mass_g = metadata["meshes"]["wheels.stl"]["mass"]
    m_wheel = wheel_mass_g * 1e-3

    data = compute_lqr_for_balancing_bot(params, total_mass - m_wheel, I_total, com, m_wheel)

    out_path = os.path.join(metadata["file_path"], "LQR_bot_data.json")
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\n✅ LQR_bot_data.json written to {out_path}")


if __name__ == "__main__":
    main()
