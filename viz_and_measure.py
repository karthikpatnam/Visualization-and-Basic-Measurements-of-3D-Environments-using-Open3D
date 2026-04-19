#!/usr/bin/env python3
"""
viz_and_measure.py
Quick Open3D visualization + simple tools for .ply point clouds (ZED 2i output).
Usage examples:
  python viz_and_measure.py --ply path/to/output_pointcloud.ply
  python viz_and_measure.py --ply scene.ply --voxel 0.02 --segment
  python viz_and_measure.py --ply scene.ply --measure   # pick points to measure distances
  python viz_and_measure.py --ply scene.ply --mesh poisson --poisson_depth 8

Features:
 - Load .ply
 - Optional voxel downsample
 - Estimate normals
 - Optional plane segmentation (RANSAC) to remove dominant plane (e.g., floor)
 - Optional mesh (poisson or ball-pivoting)
 - Interactive viewer; if --measure is used, use pick-mode to click points and compute pairwise distances
"""

import argparse
import sys
import os
import numpy as np
import open3d as o3d
from math import sqrt

def load_cloud(ply_path):
    if not os.path.isfile(ply_path):
        raise FileNotFoundError(f"{ply_path} not found")
    pcd = o3d.io.read_point_cloud(ply_path)
    if pcd.is_empty():
        raise RuntimeError(f"Loaded point cloud is empty: {ply_path}")
    return pcd

def preprocess(pcd, voxel_size):
    if voxel_size and voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
    # Estimate normals (required for meshing & some visualization effects)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2 if voxel_size else 0.05, max_nn=30))
    pcd.normalize_normals()
    return pcd

def segment_plane(pcd, distance_threshold=0.02, ransac_n=3, num_iterations=1000, keep_plane=False):
    # Returns (inliers_pcd, outliers_pcd, plane_model)
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=ransac_n,
                                             num_iterations=num_iterations)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    if keep_plane:
        return inlier_cloud, outlier_cloud, plane_model
    else:
        # usually want to remove the plane from the scene
        return inlier_cloud, outlier_cloud, plane_model

def make_mesh_poisson(pcd, depth=9, scale=1.1, linear_fit=False):
    print("[mesh] Running Poisson reconstruction, this may take time...")
    pcd_for_mesh = pcd
    if not pcd.has_normals():
        pcd_for_mesh.estimate_normals()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_for_mesh, depth=depth, scale=scale, linear_fit=linear_fit)
    # Crop by density: remove low-density vertices
    densities = np.asarray(densities)
    density_thresh = np.quantile(densities, 0.01)
    vertices_to_keep = densities > density_thresh
    mesh = mesh.remove_vertices_by_mask(~vertices_to_keep)
    mesh.compute_vertex_normals()
    return mesh

def make_mesh_ball_pivoting(pcd, radii=None):
    if not pcd.has_normals():
        pcd.estimate_normals()
    if radii is None:
        bounds = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
        radii = [bounds * r for r in (0.002, 0.006, 0.02)]
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
    mesh.compute_vertex_normals()
    return mesh

def print_stats(pcd):
    pts = np.asarray(pcd.points)
    print("Point count:", len(pts))
    aabb = pcd.get_axis_aligned_bounding_box()
    minb = aabb.min_bound
    maxb = aabb.max_bound
    extent = aabb.get_extent()
    print("AABB min:", minb)
    print("AABB max:", maxb)
    print("Extent (x,y,z):", extent)
    print("Spatial diagonal length (meters):", np.linalg.norm(extent))

def measure_with_picking(pcd):
    """
    Uses Open3D's draw_geometries_with_editing to let user pick points.
    The function returns distances between sequential picked points and prints them.
    """
    print("\n=== Measure mode ===")
    print("Instructions:")
    print("  - Left click to pick points. Press 'Q' or Esc when done.")
    print("  - The viewer will return indices of picked points (in source point cloud).")
    print("After you finish picking, distances between successive picked points will be printed.\n")
    picked = o3d.visualization.draw_geometries_with_editing([pcd], window_name="Pick points (pick at least 2)")

    if not picked:
        print("No points picked.")
        return

    try:
        picked_indices = picked  # function returns a list of indices when used this way
        picked_points = np.asarray(pcd.points)[picked_indices]
    except Exception as e:
        # fallback: try to read selection file (older open3d versions)
        selfile = "selection.json"
        if os.path.exists(selfile):
            import json
            with open(selfile, 'r') as f:
                raw = json.load(f)
                picked_indices = raw.get("picked_points", [])
                picked_points = np.asarray(pcd.points)[picked_indices]
        else:
            print("Could not obtain picked points. Exception:", e)
            return

    if len(picked_points) < 2:
        print("Picked fewer than 2 points. Nothing to measure.")
        return

    dists = []
    for i in range(1, len(picked_points)):
        d = np.linalg.norm(picked_points[i] - picked_points[i-1])
        dists.append(d)
        print(f"Distance P{i-1} -> P{i}: {d:.4f} (meters)")

    # Also print cumulative distance
    print("Cumulative distance (sum of sequential distances):", sum(dists))
    # Save picked points and distances
    np.savetxt("picked_points_xyz.txt", picked_points, header="x y z", comments='')
    with open("picked_distances.txt", "w") as f:
        for i, d in enumerate(dists):
            f.write(f"P{i}->{i+1}: {d:.6f}\n")
        f.write("Cumulative: " + str(sum(dists)) + "\n")
    print("Saved picked_points_xyz.txt and picked_distances.txt")

def main():
    parser = argparse.ArgumentParser(description="Quick Open3D viewer + tools for .ply point clouds")
    parser.add_argument("--ply", required=True, help="Path to .ply point cloud")
    parser.add_argument("--voxel", type=float, default=0.0, help="Voxel downsample size (meters), 0 to disable")
    parser.add_argument("--segment", action="store_true", help="Run plane segmentation and remove dominant plane (floor)")
    parser.add_argument("--segment_dist", type=float, default=0.02, help="RANSAC distance threshold (meters) for plane segmentation")
    parser.add_argument("--measure", action="store_true", help="Launch pick-mode to pick points and compute distances")
    parser.add_argument("--mesh", choices=["poisson", "bpa"], default=None, help="Create mesh: poisson or ball-pivoting")
    parser.add_argument("--poisson_depth", type=int, default=9, help="Poisson reconstruction depth (higher = more detail, more memory)")
    parser.add_argument("--bpa_radii", type=float, nargs="*", default=None, help="List of radii for Ball Pivoting (fractions of scene size or absolute meters)")
    args = parser.parse_args()

    pcd = load_cloud(args.ply)
    print_stats(pcd)

    pcd = preprocess(pcd, args.voxel if args.voxel > 0 else None)
    print("[info] After preprocessing:")
    print_stats(pcd)

    geometries_to_show = []

    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    geometries_to_show.append(coord_frame)

    if args.segment:
        inlier_plane, outlier_cloud, plane_model = segment_plane(pcd, distance_threshold=args.segment_dist)
        print("[segment] Plane model:", plane_model)
        print("[segment] Plane inliers count:", len(inlier_plane.points))
        print("[segment] Remaining points after removing plane:", len(outlier_cloud.points))
        # color separated clouds for clarity
        inlier_plane.paint_uniform_color([1, 0, 0])   # plane = red
        outlier_cloud.paint_uniform_color([0.7, 0.7, 0.7])
        pcd = outlier_cloud  # continue with plane removed
        geometries_to_show.append(pcd)
        geometries_to_show.append(inlier_plane)
    else:
        geometries_to_show.append(pcd)

    if args.mesh:
        if args.mesh == "poisson":
            mesh = make_mesh_poisson(pcd, depth=args.poisson_depth)
            print("[mesh] Poisson mesh created. Vertices:", len(mesh.vertices), "Triangles:", len(mesh.triangles))
            geometries_to_show = [coord_frame, mesh]
        elif args.mesh == "bpa":
            mesh = make_mesh_ball_pivoting(pcd, radii=args.bpa_radii)
            print("[mesh] BPA mesh created. Vertices:", len(mesh.vertices), "Triangles:", len(mesh.triangles))
            geometries_to_show = [coord_frame, mesh]

    # Visualize (if measure requested, we'll call the pick-mode function separately)
    if not args.measure:
        o3d.visualization.draw_geometries(geometries_to_show, window_name="Open3D - VISUALIZER", width=1280, height=720)
    else:
        # Show the point cloud first to inspect
        o3d.visualization.draw_geometries(geometries_to_show, window_name="Open3D - Inspect then pick", width=1280, height=720)
        # Now allow picking & measuring
        # For picking we must provide the original pcd (index mapping). If segmentation was done, use the current pcd
        measure_with_picking(pcd)

if __name__ == "__main__":
    main()
