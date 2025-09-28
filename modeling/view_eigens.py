#!/usr/bin/env python3
import json
import os
import sys
import trimesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    max_range = max([x_range, y_range, z_range]) / 2.0

    mid_x = np.mean(x_limits)
    mid_y = np.mean(y_limits)
    mid_z = np.mean(z_limits)

    ax.set_xlim3d([mid_x - max_range, mid_x + max_range])
    ax.set_ylim3d([mid_y - max_range, mid_y + max_range])
    ax.set_zlim3d([mid_z - max_range, mid_z + max_range])

def load_and_plot(metadata_path):
    # Load JSON metadata
    with open(metadata_path, "r") as f:
        metadata = json.load(f)

    file_path = metadata.get("file_path", ".")
    meshes_info = metadata.get("meshes", {})

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    colors = ["r", "g", "b", "c", "m", "y", "orange", "purple"]

    for i, (name, props) in enumerate(meshes_info.items()):
        stl_file = os.path.join(file_path, f"{name}.stl")
        if not os.path.exists(stl_file):
            print(f"⚠️ Missing STL: {stl_file}")
            continue

        mesh = trimesh.load(stl_file)
        if not isinstance(mesh, trimesh.Trimesh):
            print(f"⚠️ {stl_file} did not load as a mesh.")
            continue

        # Plot surface mesh (wireframe)
        color = colors[i % len(colors)]
        ax.plot_trisurf(
            mesh.vertices[:, 0], mesh.vertices[:, 1], mesh.faces, mesh.vertices[:, 2],
            linewidth=0.2, alpha=0.5, color=color
        )

        # Plot COM
        com = mesh.center_mass
        ax.scatter(com[0], com[1], com[2], color=color, s=50, marker="x")
        print(f"{name}: COM = {com}")

    # Draw joint if present
    if "joint1" in metadata:
        p1 = metadata["joint1"]["p1"]
        p2 = metadata["joint1"]["p2"]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                color="black", linewidth=3, label="joint1")
        ax.scatter(*p1, color="k", s=40)
        ax.scatter(*p2, color="k", s=40)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.legend()

    # Fix aspect ratio
    set_axes_equal(ax)

    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: ./visualize_model.py metadata.json")
        sys.exit(1)

    load_and_plot(sys.argv[1])
