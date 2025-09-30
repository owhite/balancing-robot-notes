#!/usr/bin/env python3

# This consumes the output of dump_stls.py that is launched from Rhino3D
#
# Invoke using:
# ./pybullet_generator.py pendulum_metadata.json
#
#✅Output should include:
# pendulum_metadata.xml
# pendulum.urdf 
# model_debug.json
#
# Inspect output carefully, compare moments of intertia, densities, CoMs
#  to see if they're similar to analysis tools in Rhino

import os, sys, json
import numpy as np
import trimesh
from lxml import etree as ET

def parallel_axis(I, m, d):
    """Shift inertia tensor by parallel axis theorem."""
    d = np.array(d)
    return I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

def generate_mjcf(meta, total_mass, com_rel, inertia, p1, axis, output_dir):
    mjcf = ET.Element("mujoco", model="rhino_export")
    asset = ET.SubElement(mjcf, "asset")

    for name in meta["meshes"]:
        ET.SubElement(asset, "mesh",
            name=f"{name}_mesh",
            file=f"{name}.stl",
            scale="0.001 0.001 0.001")

    worldbody = ET.SubElement(mjcf, "worldbody")

    body = ET.SubElement(worldbody, "body",
        name="pendulum",
        pos="{} {} {}".format(*p1))

    ET.SubElement(body, "inertial",
        mass=str(total_mass),
        pos="{} {} {}".format(*com_rel),
        diaginertia="{} {} {}".format(inertia[0,0], inertia[1,1], inertia[2,2]),
        quat="0 0 0 1")

    ET.SubElement(body, "joint",
        name="hinge", type="hinge",
        axis="{} {} {}".format(*axis))

    for name in meta["meshes"]:
        ET.SubElement(body, "geom", type="mesh", mesh=f"{name}_mesh")

    actuator = ET.SubElement(mjcf, "actuator")
    ET.SubElement(actuator, "motor", joint="hinge", ctrlrange="-1 1", gear="1")

    tree = ET.ElementTree(mjcf)
    tree.write(os.path.join(output_dir, "pendulum_metadata.xml"),
               pretty_print=True, xml_declaration=True, encoding="UTF-8")

def generate_urdf(meta, total_mass, com_rel, inertia, p1, axis, output_dir):
    robot = ET.Element("robot", name="pendulum")

    # Fixed base
    ET.SubElement(robot, "link", name="base_link")

    # Pendulum link
    link = ET.SubElement(robot, "link", name="pendulum_link")

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin",
        xyz="{} {} {}".format(*com_rel), rpy="0 0 0")

    ET.SubElement(inertial, "mass", value=str(total_mass))
    ET.SubElement(inertial, "inertia",
        ixx=str(inertia[0,0]), iyy=str(inertia[1,1]), izz=str(inertia[2,2]),
        ixy=str(inertia[0,1]), ixz=str(inertia[0,2]), iyz=str(inertia[1,2]))

    # Visuals and collisions — shift mesh by -p1
    for name in meta["meshes"]:
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin",
            xyz="{} {} {}".format(*(-p1)), rpy="0 0 0")
        geom = ET.SubElement(visual, "geometry")
        ET.SubElement(geom, "mesh",
            filename=f"{name}.stl", scale="0.001 0.001 0.001")

        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin",
            xyz="{} {} {}".format(*(-p1)), rpy="0 0 0")
        geomc = ET.SubElement(collision, "geometry")
        ET.SubElement(geomc, "mesh",
            filename=f"{name}.stl", scale="0.001 0.001 0.001")

    # Hinge joint
    joint = ET.SubElement(robot, "joint", name="hinge", type="revolute")
    ET.SubElement(joint, "parent", link="base_link")
    ET.SubElement(joint, "child", link="pendulum_link")
    ET.SubElement(joint, "origin",
        xyz="{} {} {}".format(*p1), rpy="0 0 0")
    ET.SubElement(joint, "axis",
        xyz="{} {} {}".format(*axis))
    ET.SubElement(joint, "limit",
        lower="-3.14", upper="3.14", effort="1000", velocity="1000")

    tree = ET.ElementTree(robot)
    tree.write(os.path.join(output_dir, "pendulum.urdf"),
               pretty_print=True, xml_declaration=True, encoding="UTF-8")

def write_debug_json(parts, total_mass, com, total_inertia, output_dir):
    debug = {
        "total_mass": float(total_mass),
        "center_of_mass": com.tolist(),
        "inertia_tensor": total_inertia.tolist(),
        "parts": {
            name: {
                "mass": float(p["mass"]),
                "com": p["com"].tolist(),
                "inertia": p["inertia"].tolist()
            } for name, p in parts.items()
        }
    }
    with open(os.path.join(output_dir, "model_debug.json"), "w") as f:
        json.dump(debug, f, indent=2)

def main():
    if len(sys.argv) < 2:
        print("Usage: pybullet_generator.py rhino_export.json")
        sys.exit(1)

    with open(sys.argv[1]) as f:
        meta = json.load(f)

    output_dir = meta["file_path"]

    total_mass = 0.0
    total_com = np.zeros(3)
    total_inertia = np.zeros((3,3))
    parts = {}

    # Process each STL
    for name, props in meta["meshes"].items():
        density = props["density"]
        stl_path = os.path.join(output_dir, f"{name}.stl")

        mesh = trimesh.load(stl_path, force='mesh')
        mesh.apply_scale(0.001)  # mm → m

        if not mesh.is_volume:
            mesh = mesh.convex_hull

        volume = mesh.volume
        mass = density * volume
        com = mesh.center_mass
        inertia = mesh.moment_inertia

        parts[name] = dict(mass=mass, com=com, inertia=inertia)

        total_mass += mass
        total_com += mass * com

    # Composite COM (world frame)
    com = total_com / total_mass

    # Extract joint info
    p1 = np.array(meta["joint1"]["p1"]) * 0.001  # mm → m
    p2 = np.array(meta["joint1"]["p2"]) * 0.001
    axis = p2 - p1
    axis = axis / np.linalg.norm(axis)

    # COM relative to hinge
    com_rel = com - p1

    # Composite inertia
    for name, props in parts.items():
        d = props["com"] - com
        total_inertia += parallel_axis(props["inertia"], props["mass"], d)

    # Emit files
    generate_mjcf(meta, total_mass, com_rel, total_inertia, p1, axis, output_dir)
    generate_urdf(meta, total_mass, com_rel, total_inertia, p1, axis, output_dir)
    write_debug_json(parts, total_mass, com, total_inertia, output_dir)

    print("✅ Wrote pendulum_metadata.xml, pendulum.urdf, and model_debug.json to", output_dir)

if __name__ == "__main__":
    main()
