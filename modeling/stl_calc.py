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

def generate_urdf(assembly_data, inertia_results, output_path="assembly.urdf"):
    """
    Generates a URDF representation for the pendulum assembly using the provided
    meshes, inertia results, and axis/origin definitions.
    """
    robot = Element("robot", name="pendulum_assembly")

    # Create a single link for the pendulum
    link = SubElement(robot, "link", name="pendulum_link")

    # Inertial block
    inertial = SubElement(link, "inertial")
    SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    SubElement(inertial, "mass", value=f"{inertia_results['total_mass']:.6f}")
    SubElement(inertial, "inertia",
               ixx="0.0", iyy="0.0", izz=f"{inertia_results['moment_of_inertia_scalar_m2']:.6e}",
               ixy="0.0", ixz="0.0", iyz="0.0")

    # Visual and collision elements for each STL
    for part in assembly_data["items"]:
        for tag_type in ["visual", "collision"]:
            tag = SubElement(link, tag_type)
            SubElement(tag, "origin", xyz="0 0 0", rpy="0 0 0")
            geom = SubElement(tag, "geometry")
            SubElement(geom, "mesh",
                       filename=os.path.basename(part["stl_file"]),
                       scale="0.001 0.001 0.001")

    # Joint connecting pendulum link to base
    joint = SubElement(robot, "joint", name="pendulum_joint", type="revolute")
    SubElement(joint, "parent", link="base_link")
    SubElement(joint, "child", link="pendulum_link")
    origin_str = " ".join(map(str, inertia_results["origin_of_rotation"]))
    axis_str = " ".join(map(str, inertia_results["axis_unit_vector"]))
    SubElement(joint, "origin", xyz=origin_str, rpy="0 0 0")
    SubElement(joint, "axis", xyz=axis_str)

    # Write XML with pretty formatting (separate lines)
    tree = ElementTree(robot)
    try:
        # Python 3.9+ supports built-in indent
        ElementTree.indent(tree, space="  ", level=0)
        tree.write(output_path, encoding="utf-8", xml_declaration=True)
    except AttributeError:
        # Manual pretty print fallback for older Python versions
        xml_str = tostring(robot, encoding="utf-8")
        parsed = minidom.parseString(xml_str)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(parsed.toprettyxml(indent="  "))
    print(f"\nURDF written to: {output_path}")


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

    # Generate URDF file in same folder
    output_urdf_path = os.path.join(base_path, "pendulum_assembly.urdf")
    generate_urdf(assembly_data, inertia_results, output_path=output_urdf_path)
