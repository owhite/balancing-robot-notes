#!/usr/bin/env python3
import sys
import os
import json
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R

TOL = 1e-8

def format_val(val):
    """Format small values as 0.0 to avoid scientific noise"""
    return 0.0 if abs(val) < TOL else float(val)

def compute_mass_properties(stl_path, density):
    """Load STL, compute volume, mass, centroid, inertia tensor."""
    mesh = trimesh.load(stl_path)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{stl_path} did not load as a Trimesh")

    volume = mesh.volume  # mm^3
    mass = volume * density  # g
    com = mesh.center_mass  # mm
    inertia = mesh.moment_inertia  # about origin, mm^5

    return volume, mass, com, inertia

def diagonalize_inertia(inertia):
    """Diagonalize inertia tensor → principal moments + quaternion orientation."""
    eigvals, eigvecs = np.linalg.eigh(inertia)

    # Ensure right-handed basis
    if np.linalg.det(eigvecs) < 0:
        eigvecs[:, 0] *= -1

    # Convert rotation matrix to quaternion
    quat = R.from_matrix(eigvecs).as_quat()  # (x, y, z, w)

    return eigvals, quat

def build_mjcf(metadata_path):
    with open(metadata_path, "r") as f:
        metadata = json.load(f)

    file_path = metadata.get("file_path", ".")
    meshes = metadata["meshes"]
    joint = metadata.get("joint1")

    total_mass = 0.0
    cx_sum = cy_sum = cz_sum = 0.0
    inertia_sum = np.zeros((3, 3))

    asset_entries = []
    geom_entries = []

    debug_info = {"parts": []}

    for name, props in meshes.items():
        stl_file = os.path.join(file_path, f"{name}.stl")
        density = props.get("density", None)
        if density is None:
            print(f"⚠️ No density for {name}, skipping")
            continue

        volume, mass_g, com, inertia = compute_mass_properties(stl_file, density)
        mass = mass_g / 1000.0  # convert g → kg
        cx, cy, cz = com / 1000.0  # mm → m

        # Accumulate
        total_mass += mass
        cx_sum += mass * cx
        cy_sum += mass * cy
        cz_sum += mass * cz
        inertia_sum += inertia / 1e12  # mm^5 → m^5

        # Add assets
        mesh_name = f"{name}_mesh"
        asset_entries.append(
            f'    <mesh name="{mesh_name}" file="{name}.stl" scale="0.001 0.001 0.001"/>'
        )
        geom_entries.append(
            f'      <geom type="mesh" mesh="{mesh_name}" rgba="0.7 0.5 0.3 1"/>'
        )

        # Debug info
        debug_info["parts"].append({
            "name": name,
            "density": density,
            "volume_mm3": volume,
            "mass_g": mass_g,
            "com_mm": com.tolist(),
            "inertia_mm5": inertia.tolist()
        })

    # Combined COM
    cx_final = cx_sum / total_mass
    cy_final = cy_sum / total_mass
    cz_final = cz_sum / total_mass

    # Diagonalize inertia
    eigvals, quat = diagonalize_inertia(inertia_sum)
    diaginertia = [format_val(v) for v in eigvals]
    quat = [format_val(q) for q in quat]  # x,y,z,w

    # Debug summary
    debug_info["total"] = {
        "mass_kg": total_mass,
        "com_m": [cx_final, cy_final, cz_final],
        "inertia_matrix_m5": inertia_sum.tolist(),
        "principal_moments": diaginertia,
        "quat": quat
    }

    with open("model_debug.json", "w") as f:
        json.dump(debug_info, f, indent=2)

    # XML output
    sb = []
    sb.append('<mujoco model="rhino_export">')
    sb.append("")
    sb.append("  <asset>")
    sb.extend(asset_entries)
    sb.append("  </asset>")
    sb.append("")
    sb.append("  <worldbody>")
    sb.append('    <body name="pendulum" pos="0 0 0">')
    sb.append(
        f'      <inertial mass="{total_mass:.6f}" pos="{cx_final:.6f} {cy_final:.6f} {cz_final:.6f}" '
        f'diaginertia="{diaginertia[0]:.6e} {diaginertia[1]:.6e} {diaginertia[2]:.6e}" '
        f'quat="{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}"/>'
    )
    if joint:
        axis = np.array(joint["p2"]) - np.array(joint["p1"])
        axis = axis / np.linalg.norm(axis)
        sb.append(
            f'      <joint name="hinge" type="hinge" axis="{axis[0]:.6f} {axis[1]:.6f} {axis[2]:.6f}"/>'
        )
    sb.extend(geom_entries)
    sb.append("    </body>")
    sb.append("  </worldbody>")
    sb.append("</mujoco>")

    return "\n".join(sb)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: ./mujoco_generator.py metadata.json")
        sys.exit(1)

    metadata_path = sys.argv[1]
    xml_out = build_mjcf(metadata_path)
    xml_file = os.path.splitext(metadata_path)[0] + ".xml"
    with open(xml_file, "w") as f:
        f.write(xml_out)
    print(f"✅ Wrote MuJoCo XML to {xml_file}")
    print("ℹ️ Debug info saved to model_debug.json")
