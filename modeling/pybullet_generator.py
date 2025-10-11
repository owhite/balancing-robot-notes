#!/usr/bin/env python3

import trimesh
import numpy as np
import sys
import json
import os
from xml.etree.ElementTree import Element, SubElement, tostring, ElementTree
from xml.dom import minidom

# Testing:
#  In this question all input units are millimeter. I have an aluminum sphere with the center at position 0,0,0 and its diameter is 10mm. I have a second steel sphere with its center at position 0,0,50 and it's diameter is 10mm. What is the moment of inertia for these combined objects for an rotation axis of [0,1,0] and a reference point on that axis of [0,0,0]?
def generate_urdf(assembly_data, inertia_results, output_path="assembly.urdf"):
    """
    Generates a URDF for the inverted pendulum assembly.
    The link frame origin coincides with the pivot (joint origin),
    while the inertial origin is offset along +Z toward the center of mass.
    Visual meshes remain centered at the pivot for correct appearance.
    """

    robot = Element("robot", name="pendulum_assembly")

    # --- Base link (massless, fixed) ---
    base_link = SubElement(robot, "link", name="base_link")
    base_inertial = SubElement(base_link, "inertial")
    SubElement(base_inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    SubElement(base_inertial, "mass", value="0.001")
    SubElement(base_inertial, "inertia",
               ixx="1e-6", iyy="1e-6", izz="1e-6", ixy="0", ixz="0", iyz="0")

    # --- Pendulum link ---
    link = SubElement(robot, "link", name="pendulum_link")
    inertial = SubElement(link, "inertial")

    # Compute COM offset (pivot → COM)
    com_mm = np.array(inertia_results["total_com"])
    pivot_mm = np.array(inertia_results["origin_of_rotation"])
    com_offset_m = (com_mm - pivot_mm) * 0.001
    m = inertia_results["total_mass"]
    I_com = inertia_results["moment_of_inertia_scalar_m2"]

    print(f"✅ Pivot→COM offset (m): {com_offset_m}")

    # Inertia tensor about COM (URDF expects inertia about COM)
    SubElement(inertial, "origin",
               xyz=f"{com_offset_m[0]:.6f} {com_offset_m[1]:.6f} {com_offset_m[2]:.6f}",
               rpy="0 0 0")
    SubElement(inertial, "mass", value=f"{m:.6f}")
    SubElement(inertial, "inertia",
               ixx="1e-6", iyy=f"{I_com:.6e}", izz="1e-6",
               ixy="0.0", ixz="0.0", iyz="0.0")

    # --- Visual and collision meshes (centered at pivot) ---
    for part in assembly_data["items"]:
        for tag_type in ["visual", "collision"]:
            tag = SubElement(link, tag_type)
            SubElement(tag, "origin", xyz="0 0 0", rpy="0 0 0")  # pivot-centered
            geom = SubElement(tag, "geometry")
            SubElement(geom, "mesh",
                       filename=os.path.basename(part["stl_file"]),
                       scale="0.001 0.001 0.001")

    # --- Revolute joint ---
    joint = SubElement(robot, "joint", name="pendulum_joint", type="revolute")
    SubElement(joint, "parent", link="base_link")
    SubElement(joint, "child", link="pendulum_link")

    joint_origin_m = np.array(inertia_results["origin_of_rotation"]) * 0.001
    SubElement(joint, "origin",
               xyz=f"{joint_origin_m[0]:.6f} {joint_origin_m[1]:.6f} {joint_origin_m[2]:.6f}",
               rpy="0 0 0")
    SubElement(joint, "axis",
               xyz=" ".join(map(str, inertia_results["axis_unit_vector"])))
    SubElement(joint, "limit",
               lower="-3.1416", upper="3.1416", effort="1", velocity="1")

    tree = ElementTree(robot)
    # --- Write XML with pretty formatting (works for all Python versions) ---
    tree = ElementTree(robot)
    try:
        # Only available in Python 3.9+
        ElementTree.indent(tree, space="  ", level=0)
        tree.write(output_path, encoding="utf-8", xml_declaration=True)
    except AttributeError:
        # Manual pretty-print fallback
        xml_str = tostring(robot, encoding="utf-8")
        parsed = minidom.parseString(xml_str)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(parsed.toprettyxml(indent="  "))

    print(f"✅ URDF written to: {output_path}")


def calculate_total_moment_of_inertia(assembly_data: dict):
    """
    Calculates the total moment of inertia for an assembly of parts,
    handling multiple disconnected meshes within an STL file and
    different densities for each specified part.

    Args:
        assembly_data: A dictionary containing the parameters for the entire assembly,
                       including the list of parts, axis, and origin of rotation.
    """
    total_scalar_moment_of_inertia = 0.0
    total_mass = 0.0
    weighted_com_sum = np.zeros(3)

    # Extract assembly-wide parameters
    axis_of_rotation = np.array(assembly_data["axis_of_rotation"])
    origin_of_rotation = np.array(assembly_data["origin_of_rotation"])

    # Ensure the axis of rotation is a unit vector
    axis_unit_vector = axis_of_rotation / np.linalg.norm(axis_of_rotation)

    # --- 1. Calculate mass and inertia for each individual part file ---
    for part in assembly_data["items"]:
        stl_file_path = part["stl_file"]
        print("STL file: ", stl_file_path)

        try:
            full_mesh = trimesh.load_mesh(stl_file_path)
        except FileNotFoundError:
            print(f"Error: STL file not found at '{stl_file_path}'. Exiting gracefully.")
            sys.exit(1)

        # Split the mesh into its constituent sub-parts
        sub_meshes = full_mesh.split(only_watertight=False)
        count = 0
        sub_mass = 0.0
        sub_weighted_com_sum = np.zeros(3)

        for sub_mesh in sub_meshes:
            # Apply any additional intended position translation to each sub-part.
            sub_mesh.apply_translation(part.get("part_position_mm", np.zeros(3)))

            # Calculate properties for this sub-part
            density = part["density_kg_per_mm3"]
            mass_kg = sub_mesh.volume * density
            inertia_tensor_com = sub_mesh.moment_inertia * density
            center_of_mass_mm = sub_mesh.center_mass

            # Project the part's inertia tensor (relative to its own COM) onto the axis direction.
            scalar_inertia_com = axis_unit_vector.T @ inertia_tensor_com @ axis_unit_vector

            # Calculate the perpendicular distance from the part's COM to the rotation axis
            # The rotation axis is defined by origin_of_rotation and axis_unit_vector
            com_to_axis_origin = center_of_mass_mm - origin_of_rotation
            distance_vector = com_to_axis_origin - (com_to_axis_origin @ axis_unit_vector) * axis_unit_vector
            distance_mm = np.linalg.norm(distance_vector)
            
            # Apply the scalar parallel axis theorem
            scalar_inertia_part = scalar_inertia_com + mass_kg * (distance_mm ** 2)

            # Sum up the properties for the sub-parts within this STL file
            sub_mass += mass_kg
            sub_weighted_com_sum += center_of_mass_mm * mass_kg

            # Sum the properties for the entire assembly
            total_mass += mass_kg
            weighted_com_sum += center_of_mass_mm * mass_kg
            total_scalar_moment_of_inertia += scalar_inertia_part
            count += 1
        
        sub_com = sub_weighted_com_sum / sub_mass if sub_mass > 0 else np.zeros(3)
        print(f"Sub-parts in STL: {count} total mass: {sub_mass * 1000:.4f} (gram) total CoM: {sub_com}")

    # --- 2. Calculate the overall center of mass ---
    if total_mass > 0:
        total_com = weighted_com_sum / total_mass
    else:
        print("mass problem encountered: Total mass is zero.")
        total_com = np.zeros(3)

    # --- 3. Output the final results ---
    moment_of_inertia_scalar_m2 = total_scalar_moment_of_inertia * 1e-6
    
    print("\n--- Total Inertia Calculation for all parts ---")
    print(f"Total mass (in kg): {total_mass}")
    print(f"Total center of mass (in mm): {total_com}")
    print(f"Origin of rotation (in mm): {origin_of_rotation}")
    print(f"Axis of rotation (unit vector): {axis_unit_vector}")
    print(f"\nCalculated Total Moment of Inertia about the axis (in kg·m²): {moment_of_inertia_scalar_m2}")

    # Return useful data for URDF generation
    return {
        "total_mass": total_mass,
        "total_com": total_com,
        "origin_of_rotation": origin_of_rotation,
        "axis_unit_vector": axis_unit_vector,
        "moment_of_inertia_scalar_m2": moment_of_inertia_scalar_m2
    }

if __name__ == "__main__":
    with open(sys.argv[1], "r") as f:
        config = json.load(f)

    base_path = config["file_path"]
    items = []        
    assembly_data = []

    # note that axis_of_rotation is a vector, it is not a line going through space.
    #   origin_of_rotation is a point in space

    assembly_data = {
        "axis_of_rotation": np.array(config["axis_of_rotation"]),
        "origin_of_rotation": np.array(config["origin"]),
        "items": items
    }

    for filename, mesh_info in config["meshes"].items():
        items.append({
            "stl_file": os.path.join(base_path, filename),
            "density_kg_per_mm3": mesh_info["density"] * 1e-9  # kg/m³ → kg/mm³
        })

    inertia_results = calculate_total_moment_of_inertia(assembly_data)

    # --- Generate URDF file in same folder ---
    output_urdf_path = os.path.join(base_path, "pendulum_assembly.urdf")
    generate_urdf(assembly_data, inertia_results, output_path=output_urdf_path)

    # --- Generate JSON file with variables for LQR and debugging ---
    m = inertia_results["total_mass"]
    I = inertia_results["moment_of_inertia_scalar_m2"]
    total_com = inertia_results["total_com"]
    origin = inertia_results["origin_of_rotation"]
    r = np.linalg.norm(total_com - origin) * 1e-3  # mm→m
    g = 9.81
    mgr_over_I = (m * g * r) / I
    omega_n = np.sqrt(mgr_over_I) if mgr_over_I > 0 else 0.0
    period_T = 2 * np.pi / omega_n if omega_n > 0 else 0.0
    A = [[0, 1], [mgr_over_I, 0]]
    B = [[0], [1 / I]]

    debug_data = {
        "mass_kg": m,
        "moment_of_inertia_kg_m2": I,
        "total_com_mm": total_com.tolist(),
        "origin_of_rotation_mm": origin.tolist(),
        "r_m": r,
        "g_m_per_s2": g,
        "mgr_over_I": mgr_over_I,
        "omega_n_rad_per_s": omega_n,
        "expected_period_s": period_T,
        "axis_unit_vector": inertia_results["axis_unit_vector"].tolist(),
        "A_matrix": A,
        "B_matrix": B,
        "urdf_file": output_urdf_path
    }

    json_output_path = os.path.join(base_path, "pendulum_LQR_data.json")
    with open(json_output_path, "w") as f:
        json.dump(debug_data, f, indent=2)

    print(f"\nLQR variable data written to: {json_output_path}")
    print(f"\nComputed ω_n = {omega_n:.3f} rad/s, T = {period_T:.3f} s, mgr/I = {mgr_over_I:.3f}")
