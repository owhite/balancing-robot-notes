#!/usr/bin/env python3
"""
Generate LQR gains and physical parameters for a torque-controlled balancing robot.

Usage:
    ./generate_TWR_data.py -m LQR_bot_metadata.json -p robot_params.json

Updates:
  • Excludes wheels from COM and inertia when computing body parameters (l).
  • Uses per-part masses (grams) from STL metadata, not densities.
  • Incorporates axle origin and direction from metadata to compute
    perpendicular distance to COM correctly.
  • Builds a 4-state torque-controlled inverted-pendulum-on-wheels model.
  • Reads motor parameters separately (robot_params.json).
  • Outputs LQR_bot_data.json in the same folder as the metadata file.

This is based on a CAD diagram where:
  • The wheels rest on the X–Y plane, so Z = 0 corresponds to the ground surface.
  • The axle runs along the Y-axis, 
  • In this case, axle is through the point (0, 0, 40 mm)
  • The axis vector is (0, 1, 0)
  • The robot body extends mainly along +X (forward direction).
  • The center of mass (COM) lies somewhere above the axle (along +Z) and behind or ahead along X.
  • Pitching motion (balancing) occurs in the X–Z plane about the Y-axis.
  • Export of STL meshes is in mm. 
"""

import json
import numpy as np
import argparse
import os
import trimesh
from scipy.linalg import solve_continuous_are, solve_discrete_are, expm


def calculate_total_inertia_from_mass(assembly_data: dict):
    """Compute total mass, center of mass, and scalar moment of inertia."""
    total_mass = 0.0
    weighted_com_sum = np.zeros(3)
    total_scalar_inertia = 0.0

    for name, part in assembly_data["meshes"].items():
        mass_g = part["mass"]
        stl_path = os.path.join(assembly_data["file_path"], name)
        mesh = trimesh.load_mesh(stl_path)
        mesh_mass_kg = mass_g * 1e-3

        I_tensor = mesh.moment_inertia * (mesh_mass_kg / mesh.mass)
        com_mm = mesh.center_mass
        origin = np.zeros(3)

        r_vec = com_mm - origin
        d = np.linalg.norm(r_vec)
        I_axis = (np.trace(I_tensor) / 3.0) * 1e-6 + mesh_mass_kg * (d * 1e-3) ** 2

        total_mass += mesh_mass_kg
        weighted_com_sum += com_mm * mesh_mass_kg
        total_scalar_inertia += I_axis

    total_com = weighted_com_sum / total_mass
    I_total_kg_m2 = total_scalar_inertia
    return total_mass, total_com, I_total_kg_m2

def compute_lqr_for_balancing_bot(params, m_body, I_body, com_mm, m_wheel, axle_origin_mm, axle_vector):
    """Compute continuous and discrete LQR gains for torque-controlled balancing robot."""
    g = 9.81
    axle_unit = axle_vector / np.linalg.norm(axle_vector)

    # Compute perpendicular distance from axle to COM
    r_vec = com_mm - axle_origin_mm
    r_perp = r_vec - np.dot(r_vec, axle_unit) * axle_unit
    l = np.linalg.norm(r_perp) * 1e-3  # convert mm → m

    r = params["wheel_radius"] * 1e-3  # wheel radius (m)
    b_total = params["b_Nm_s_per_rad"] + params["friction_term"]

    print("\n=== PHYSICAL PARAMETERS ===")
    print(f"Body mass       m_b = {m_body:.4f} kg")
    print(f"Wheel mass      m_w = {m_wheel:.4f} kg")
    print(f"Moment inertia  I_b = {I_body:.4e} kg·m²")
    print(f"CoM (mm)        = {com_mm}")
    print(f"Axle origin (mm)= {axle_origin_mm}")
    print(f"Axle vector     = {axle_vector}")
    print(f"Distance l      = {l:.4f} m")
    print(f"Wheel radius    = {r:.4f} m")
    print(f"Total damping   = {b_total:.4f} N·m·s/rad")

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

    print(f"opening: {args.metadata}")
    with open(args.metadata, "r") as f:
        metadata = json.load(f)

    print(f"opening: {args.params}")
    with open(args.params, "r") as f:
        param_file = json.load(f)
    params = param_file["params"]

    # --- Axle geometry from metadata ---
    axle_origin_mm = np.array(metadata.get("axle_origin", [0.0, 0.0, 0.0]))
    axle_vector = np.array(metadata.get("axle_vector", [0.0, 1.0, 0.0]))

    # --- Calculate total (body + wheels) for reference ---
    total_mass, com_total, I_total = calculate_total_inertia_from_mass(metadata)

    # --- Exclude wheels for body-only computation ---
    nonwheel_parts = {k: v for k, v in metadata["meshes"].items() if k != "wheels.stl"}
    body_metadata = {"meshes": nonwheel_parts, "file_path": metadata["file_path"]}
    body_mass, com_body, I_body = calculate_total_inertia_from_mass(body_metadata)

    # Identify wheel mass
    wheel_mass_g = metadata["meshes"]["wheels.stl"]["mass"]
    m_wheel = wheel_mass_g * 1e-3

    data = compute_lqr_for_balancing_bot(params, body_mass, I_body, com_body, m_wheel, axle_origin_mm, axle_vector)

    out_path = os.path.join(metadata["file_path"], "LQR_bot_data.json")
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\n✅ LQR_bot_data.json written to {out_path}")


if __name__ == "__main__":
    main()
