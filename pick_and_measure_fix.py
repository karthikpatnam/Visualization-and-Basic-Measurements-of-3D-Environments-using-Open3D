#!/usr/bin/env python3
"""
pick_and_measure_fix.py

Robust pick-and-measure helper for Open3D.

Usage:
  python pick_and_measure_fix.py --ply "P:\AIforRobotics\zed_output\scene1.ply"

How it works:
 - Opens a pick UI via VisualizerWithEditing (this writes selection.json)
 - After you press Q or close the window, it parses selection.json and prints picked indices, coordinates, pairwise distances, and saves outputs.
 - If no picks found, prints a clear message why.
"""

import os, sys, argparse, json
import numpy as np
import open3d as o3d

SELECTION_FILE = "selection.json"

def load_pcd(path):
    if not os.path.isfile(path):
        raise FileNotFoundError(f"{path} not found")
    pcd = o3d.io.read_point_cloud(path)
    if pcd.is_empty():
        raise RuntimeError("Point cloud is empty")
    return pcd

def launch_picker(pcd):
    """
    Launch VisualizerWithEditing which writes selection.json on exit.
    This is more reliable across Open3D versions than draw_geometries_with_editing returning indices.
    """
    # ensure previous selection file removed so we know new one is from this session
    if os.path.exists(SELECTION_FILE):
        try:
            os.remove(SELECTION_FILE)
        except:
            pass
    print("Viewer: left-click to pick points. Use mouse wheel to zoom. Press 'Q' or Esc to finish.")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Pick points - left click to pick, Q/Esc to finish", width=1280, height=720)
    vis.add_geometry(pcd)
    vis.run()  # blocks until window closed
    vis.destroy_window()
    print("Viewer closed. Checking for selection file:", SELECTION_FILE)

def read_selection_file():
    if not os.path.exists(SELECTION_FILE):
        return None
    try:
        with open(SELECTION_FILE, 'r') as f:
            j = json.load(f)
        # Try to find list of indices in common keys
        for key in ("picked_points","picked_indices","indices"):
            if key in j and isinstance(j[key], list):
                return j[key]
        # Sometimes the json stores them under arrays or deeper; attempt to flatten ints
        flat = []
        def collect(obj):
            if isinstance(obj, dict):
                for v in obj.values():
                    collect(v)
            elif isinstance(obj, list):
                for v in obj:
                    collect(v)
            elif isinstance(obj, int):
                flat.append(obj)
        collect(j)
        if flat:
            return flat
    except Exception as e:
        print("Error reading selection.json:", e)
    return None

def print_and_save_results(pcd, indices):
    pts = np.asarray(pcd.points)
    picked_pts = pts[indices]
    print("Picked indices:", indices)
    print("Picked coordinates (x y z):")
    for i,p in enumerate(picked_pts):
        print(f"P{i}: {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    # pairwise sequential distances
    if len(picked_pts) >= 2:
        dists = []
        for i in range(1, len(picked_pts)):
            d = np.linalg.norm(picked_pts[i] - picked_pts[i-1])
            dists.append(d)
            print(f"Distance P{i-1} -> P{i}: {d:.4f} m")
        print("Cumulative distance:", sum(dists), "m")
        np.savetxt("picked_points_xyz.txt", picked_pts, header="x y z", comments='')
        with open("picked_distances.txt","w") as f:
            for i,d in enumerate(dists):
                f.write(f"P{i}->{i+1}: {d:.6f}\n")
            f.write("Cumulative: " + str(sum(dists)) + "\n")
        print("Saved picked_points_xyz.txt and picked_distances.txt")
    else:
        print("Need at least 2 picked points to compute distances. Pick more points and try again.")
        if len(picked_pts) == 1:
            np.savetxt("picked_points_xyz.txt", picked_pts, header="x y z", comments='')
            print("Saved single picked point to picked_points_xyz.txt")

def main():
    parser = argparse.ArgumentParser(description="Robust pick-and-measure with Open3D (writes selection.json)")
    parser.add_argument("--ply", required=True, help="path to .ply")
    args = parser.parse_args()

    pcd = load_pcd(args.ply)
    launch_picker(pcd)
    indices = read_selection_file()
    if indices is None or len(indices) == 0:
        print("No points were picked or selection.json not found. Possible reasons:")
        print(" - You closed the viewer without clicking points.")
        print(" - You clicked but clicks missed points (try zooming or increase point size in viewer).")
        print(" - selection.json was written to a different working directory.")
        print("\nCheck your current working dir:", os.getcwd())
        print("If selection.json exists elsewhere, move it here or run script from that folder.")
        sys.exit(1)
    # ensure indices are ints
    indices = [int(i) for i in indices]
    print_and_save_results(pcd, indices)

if __name__ == '__main__':
    main()
