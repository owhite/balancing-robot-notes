#!/usr/bin/env python3
"""
Compute the physical parameters and linearized state-space model
for an inverted pendulum from its CAD-derived JSON configuration.

This script produces pendulum_LQR_data.json containing:
  - mass, moment of inertia, COM, lever arm
  - mgr/I term
  - natural frequency and period
  - A and B matrices for LQR design

All distances in the input are in millimeters.
"""

import json
import numpy as np
import os
import sys
import trimesh

def calculate_total_moment_of_inertia(assembly_data: dict):
    """Compute total mass, center of mass, and moment of inertia about the given axis."""
    total_scalar_inertia = 0.0
    total_mass = 0.0
    weighted_com_sum = np.zeros(3)

    axis = np.array(assembly_data["axis_of_rotation"], dtype=float)
    origin = np.array(assembly_data["origin_of_rotation"], dtype=float)
    axis_unit = axis / np.linalg.norm(axis)

    for part in assembly_data["items"]:
        stl_path = part["stl_file"]
        print(f"🔹 Loading {stl_path}")
        mesh = trimesh.load_mesh(stl_path)
        sub_meshes = mesh.split(only_watertight=False)

        for sub_mesh in sub_meshes:
            sub_mesh.apply_translation(part.get("part_position_mm", np.zeros(3)))
            density = part["density_kg_per_mm3"]
            mass_kg = sub_mesh.volume * density
            I_tensor = sub_mesh.moment_inertia * density
            com_mm = sub_mesh.center_mass

            # Inertia projected along rotation axis
            I_axis_com = axis_unit @ I_tensor @ axis_unit

            # Parallel-axis term
            r_vec = com_mm - origin
            r_perp = r_vec - (r_vec @ axis_unit) * axis_unit
            d = np.linalg.norm(r_perp)
            I_axis = I_axis_com + mass_kg * (d ** 2)

            total_mass += mass_kg
            weighted_com_sum += com_mm * mass_kg
            total_scalar_inertia += I_axis

    if total_mass == 0:
        raise ValueError("Total mass is zero; check densities or STL paths.")

    total_com = weighted_com_sum / total_mass
    I_total_m2 = total_scalar_inertia * 1e-6  # mm² → m²
    print(f"✅ Total mass: {total_mass:.6f} kg")
    print(f"✅ Total COM:  {total_com} mm")
    print(f"✅ I_total:    {I_total_m2:.6e} kg·m²")

    return {
        "total_mass": total_mass,
        "total_com": total_com,
        "origin_of_rotation": origin,
        "axis_unit_vector": axis_unit,
        "moment_of_inertia_scalar_m2": I_total_m2
    }


def compute_lqr_parameters(inertia_results, base_path):
    """Compute all values that go into pendulum_LQR_data.json."""
    m = inertia_results["total_mass"]
    I = inertia_results["moment_of_inertia_scalar_m2"]
    com = inertia_results["total_com"]
    origin = inertia_results["origin_of_rotation"]
    r = np.linalg.norm(com - origin) * 1e-3  # mm → m
    g = 9.81
    mgr_over_I = (m * g * r) / I
    omega_n = np.sqrt(mgr_over_I)
    period_T = 2 * np.pi / omega_n if omega_n > 0 else np.inf

    A = [[0, 1], [mgr_over_I, 0]]
    B = [[0], [1 / I]]

    print("\n--- Computed LQR parameters ---")
    print(f"mass m = {m:.6f} kg")
    print(f"I = {I:.6e} kg·m²")
    print(f"r = {r:.6f} m")
    print(f"mgr/I = {mgr_over_I:.6f}")
    print(f"ω_n = {omega_n:.6f} rad/s,  T = {period_T:.6f} s")

    return {
        "mass_kg": m,
        "moment_of_inertia_kg_m2": I,
        "total_com_mm": com.tolist(),
        "origin_of_rotation_mm": origin.tolist(),
        "r_m": r,
        "g_m_per_s2": g,
        "mgr_over_I": mgr_over_I,
        "omega_n_rad_per_s": omega_n,
        "expected_period_s": period_T,
        "axis_unit_vector": inertia_results["axis_unit_vector"].tolist(),
        "A_matrix": A,
        "B_matrix": B,
        "urdf_file": os.path.join(base_path, "pendulum_assembly.urdf")
    }


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./generate_pendulum_LQR_data.py <assembly_config.json>")
        sys.exit(1)

    with open(sys.argv[1], "r") as f:
        config = json.load(f)

    base_path = config["file_path"]
    items = []
    for filename, mesh_info in config["meshes"].items():
        items.append({
            "stl_file": os.path.join(base_path, filename),
            "density_kg_per_mm3": mesh_info["density"] * 1e-9  # convert kg/m³ → kg/mm³
        })

    assembly_data = {
        "axis_of_rotation": np.array(config["axis_of_rotation"]),
        "origin_of_rotation": np.array(config["origin"]),
        "items": items
    }

    inertia_results = calculate_total_moment_of_inertia(assembly_data)
    data = compute_lqr_parameters(inertia_results, base_path)

    out_path = os.path.join(base_path, "pendulum_LQR_data.json")
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\n✅ pendulum_LQR_data.json written to {out_path}")
