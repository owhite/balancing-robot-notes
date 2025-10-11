# -*- coding: utf-8 -*-
#
# This code resides in the Rhino 3d scripts directory:
#  /Users/USERNAME/Library/Application Support/McNeel/Rhinoceros/7.0/scripts
#
# For parts of different densities. Select all parts, right mouse click
#  select "object properties" and fill in the name. The name then corresponds to
#  "Window->Floating panels->Document user text panel". Go to
#  document user text tab, fill in key value pairs with name and
#  density in (kg/m³). (e.g., steel is 7850). 
# Also include:
#   "file_path" in the user text panel
#   "origin"
#   "axis_of_rotation"
# in the document. 
#
# Output will be one STL for all polysurfaces that are named the same
#  and a filename_metadata.json that has densities and hinge. 
#  These are consumed later by other modeling scripts. 
# Note that pybullet only likes binary STLs

import os
import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc
import System
import json

def export_stls_and_metadata():
    # --- Get file path from doc user text ---
    file_path = rs.GetDocumentUserText("file_path")
    if not file_path:
        print("No 'file_path' key set in document user text.")
        return
    if not os.path.exists(file_path):
        os.makedirs(file_path)

    # --- Parse densities ---
    densities = {}
    keys = rs.GetDocumentUserText()
    if keys:
        for key in keys:
            if key == "file_path":
                continue
            try:
                densities[key] = float(rs.GetDocumentUserText(key))  # kg/m^3
            except:
                pass

    # --- Group objects by name ---
    objs = rs.AllObjects()
    by_name = {}
    for obj_id in objs:
        name = rs.ObjectName(obj_id)
        if not name:
            continue
        if name.lower().startswith("joint"):  # skip joints for STL export
            continue
        by_name.setdefault(name, []).append(obj_id)

    metadata = {
        "file_path": file_path,  # ✅ included
        "meshes": {}
    }

    # --- Export each unique name as one STL ---
    for name, members in by_name.items():
        stl_name = "{}.stl".format(name)
        stl_path = os.path.join(file_path, stl_name)

        rs.SelectObjects(members)
        # silent binary STL export
        rs.Command('_-Export "{}" _Enter _Binary _Enter'.format(stl_path), echo=False)
        rs.UnselectAllObjects()

        density = densities.get(name, None)
        metadata["meshes"][name + ".stl"] = {"density": density}

    # --- add axis_of_rotation ---
    axis_str = rs.GetDocumentUserText("axis_of_rotation")
    if axis_str:
        try:
            # split string like "0, 1, 0" → [0.0, 1.0, 0.0]
            axis_vals = [float(v.strip()) for v in axis_str.split(",")]
            metadata["axis_of_rotation"] = axis_vals
        except:
            print("Warning: Could not parse 'axis_of_rotation' string.")
    else:
        print("No 'axis_of_rotation' key set in document user text.")

    # --- add origin (optional, if present) ---
    origin_str = rs.GetDocumentUserText("origin")
    if origin_str:
        try:
            origin_vals = [float(v.strip()) for v in origin_str.split(",")]
            metadata["origin"] = origin_vals
        except:
            print("Warning: Could not parse 'origin' string.")
    else:
        print("No 'origin' key set in document user text.")

    # --- Save metadata JSON ---
    cad_filename = rs.DocumentName() or "model"
    base, _ = os.path.splitext(cad_filename)
    json_path = os.path.join(file_path, base + "_metadata.json")

    with open(json_path, "w") as f:
        json.dump(metadata, f, indent=2)

    print("[DONE] Exported binary STLs and metadata JSON -> {}".format(json_path))


if __name__ == "__main__":
    export_stls_and_metadata()
