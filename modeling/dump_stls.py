# -*- coding: utf-8 -*-
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
                densities[key] = float(rs.GetDocumentUserText(key))  # g/mm^3
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
        "file_path": file_path,  # ✅ now included
        "meshes": {}
    }

    # --- Export each unique name as one STL ---
    for name, members in by_name.items():
        stl_name = "{}.stl".format(name)
        stl_path = os.path.join(file_path, stl_name)

        rs.SelectObjects(members)
        # silent binary STL export
        rs.Command('_-Export "{}" _ExportFileAs=_Binary _Enter _Binary _Enter'.format(stl_path), echo=False)
        rs.UnselectAllObjects()

        density = densities.get(name, None)
        metadata["meshes"][name] = {"density": density}

    # --- Look for a joint curve named "joint1" ---
    joint_obj = None
    for obj_id in objs:
        if rs.ObjectName(obj_id) == "joint1":
            joint_obj = obj_id
            break
    if joint_obj:
        pts = rs.CurvePoints(joint_obj)
        if len(pts) == 2:
            metadata["joint1"] = {
                "p1": [pts[0].X, pts[0].Y, pts[0].Z],
                "p2": [pts[1].X, pts[1].Y, pts[1].Z]
            }

    # --- Save metadata JSON ---
    cad_filename = rs.DocumentName() or "model"
    base, _ = os.path.splitext(cad_filename)
    json_path = os.path.join(file_path, base + "_metadata.json")

    with open(json_path, "w") as f:
        json.dump(metadata, f, indent=2)

    print("[DONE] Exported binary STLs and metadata JSON -> {}".format(json_path))


if __name__ == "__main__":
    export_stls_and_metadata()
