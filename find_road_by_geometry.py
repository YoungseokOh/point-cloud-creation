# [New Version: Euclidean Distance Method]
# This script has been completely rewritten based on user feedback.
# The new approach finds the closest line of points to the LiDAR origin (0,0,0)
# based on the true 3D Euclidean distance.
# All complex filtering (XY radius, X/Y-axis limits) has been removed to create a more
# direct and intuitive result.

import open3d as o3d
import numpy as np
import os
import sys
import argparse # Added for command-line argument parsing
import json  # Added for JSON saving

# Global variables for visualization
vis = None
current_pcd_idx = 0
pcd_files = []
pcd_display_geometry = None
origin_sphere_geometry = None

# NEW: marker sphere for the selected purple point
selected_marker_geometry = None
selected_marker_pos = None
last_selected_marker_pos = None

# NEW: line geometries (black x-axis segment, light-green distance line)
x_axis_line_geometry = None
distance_line_geometry = None
# NEW: vertical purple z-line at black line end
vertical_purple_line_geometry = None

# NEW: fixed blue depth line and marker at (0,0,-0.771)
blue_depth_line_geometry = None
blue_marker_geometry = None
last_blue_marker_pos = None
BLUE_DEPTH_Z = -0.771

# NEW: g yellow vertical segment (0,0,BLUE_DEPTH_Z) -> (0,0,z_purple)
yellow_g_line_geometry = None
# NEW: light-green line from blue point to the purple line at z=z_purple
blue_to_purple_line_geometry = None
# 선 연장 배율(>1이면 보라색 점을 넘어 연장)
GREEN_EXTEND_FACTOR = 10.0

# NEW: angle arc geometries (all black)
arc_d_geometry = None  # arc for d = angle(a,b)
arc_e_geometry = None  # arc for e = angle(c,b)
arc_x_geometry = None  # arc for x = d - e
# Toggle to draw angle arcs (off for now while validating 'd')
DRAW_ANGLE_ARCS = False

# NEW: circles in the (b,z) plane centered at blue point using radii along c
c_circles_geometry = None
# Previous method circles (ground plane) for comparison
c_circles_prev_geometry = None
NUM_C_RADII = 25            # 더 촘촘하게 링 생성
CIRCLE_SEGS = 128
SKIP_FAR_COUNT = 1         # 먼쪽(보라색점 방향)에서 N개 링 제거, 가까운 쪽은 살림
RING_RADIUS_SCALE = 0.15
RING_RADIUS_MIN = 0.02
RING_RADIUS_MAX = 0.50
NEAR_BIAS = 1.2
MIN_FIRST_RADIUS = 0.01
# NEW: radii distribution along c
RADIUS_DISTRIBUTION = "cosine"  # 'uniform' | 'near' | 'cosine' (cosine: 양 끝이 더 촘촘)
# NEW: options
DRAW_PREV_CIRCLES = False   # 빨간색 방식은 비교를 위해 기본 끔(요청: 주석처리 대체)
c_circles_pcd_geometry = None  # 원 포인트들을 모아 PCD로 표시
XY_MIN_SEPARATION = 0.1
# Ground grid (z=0) toggle
ground_plane_geometry = None
GROUND_SIZE_X = 20.0  # meters (extends to ±20m in X)
GROUND_SIZE_Y = 20.0  # meters (extends to ±20m in Y)
GROUND_STEP = 1.0     # grid spacing in meters

def find_closest_line_by_euclidean_distance(
    pcd_original, # Now takes pcd_original directly
    ground_z_min=-1.5,  # Z-filter: Minimum height to be considered ground
    ground_z_max=0.5,   # Z-filter: Maximum height to be considered ground
    min_xy_distance_from_origin=0.0, # New: Minimum horizontal distance from origin
    xy_radius_threshold=np.inf, # New: Maximum horizontal distance from origin
    exclude_positive_x=True, # New: Exclude points with positive X-coordinates
    angular_resolution=1.0, # Degrees per bin for finding closest points
    y_zero_band=0.01  # NEW: highlight |y| <= band in purple
):
    """
    Loads a PCD, applies a simple Z-height filter, then finds the single closest point
    to the origin for each angular division based on 3D Euclidean distance.
    Returns the display PCD, origin sphere, and closest line points for external visualization.
    """
    if not pcd_original.has_points():
        print("Error: The point cloud is empty.")
        return None, None, None
        
    points = np.asarray(pcd_original.points)

    # 2. Apply simple Z-axis filtering to get ground candidates
    ground_indices = np.where(
        (points[:, 2] > ground_z_min) & (points[:, 2] < ground_z_max)
    )[0]
    
    if len(ground_indices) == 0:
        print(f"Error: No ground candidate points found in Z-range ({ground_z_min} to {ground_z_max}).")
        print("Please adjust the ground_z_min and ground_z_max parameters.")
        # For visualization, return original PCD as display and a dummy sphere
        pcd_display = o3d.geometry.PointCloud()
        pcd_display.points = pcd_original.points
        pcd_display.paint_uniform_color([0.5, 0.5, 0.5]) # Gray
        origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        origin_sphere.paint_uniform_color([1, 1, 0]) # Yellow
        return pcd_display, origin_sphere, None

    ground_points = points[ground_indices]

    # Apply XY distance filter to remove points too close to the origin
    if min_xy_distance_from_origin > 0:
        xy_distances = np.linalg.norm(ground_points[:, :2], axis=1)
        xy_filtered_indices = np.where(xy_distances > min_xy_distance_from_origin)[0]
        
        if len(xy_filtered_indices) == 0:
            print(f"Error: No ground candidate points found after XY distance filter (min_xy_distance_from_origin={min_xy_distance_from_origin}).")
            pcd_display = o3d.geometry.PointCloud()
            pcd_display.points = pcd_original.points
            pcd_display.paint_uniform_color([0.5, 0.5, 0.5]) # Gray
            origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
            origin_sphere.paint_uniform_color([1, 1, 0]) # Yellow
            return pcd_display, origin_sphere, None
        
        ground_points = ground_points[xy_filtered_indices]
        ground_indices = ground_indices[xy_filtered_indices] # Update original indices as well

    # Apply XY radius threshold filter to remove points too far from the origin
    if xy_radius_threshold < np.inf:
        xy_distances = np.linalg.norm(ground_points[:, :2], axis=1)
        xy_radius_filtered_indices = np.where(xy_distances < xy_radius_threshold)[0]

        if len(xy_radius_filtered_indices) == 0:
            print(f"Error: No ground candidate points found after XY radius threshold filter (xy_radius_threshold={xy_radius_threshold}).")
            pcd_display = o3d.geometry.PointCloud()
            pcd_display.points = pcd_original.points
            pcd_display.paint_uniform_color([0.5, 0.5, 0.5]) # Gray
            origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
            origin_sphere.paint_uniform_color([1, 1, 0]) # Yellow
            return pcd_display, origin_sphere, None

        ground_points = ground_points[xy_radius_filtered_indices]
        ground_indices = ground_indices[xy_radius_filtered_indices] # Update original indices as well

    # Apply X-axis positive exclusion filter
    if exclude_positive_x:
        x_filtered_indices = np.where(ground_points[:, 0] <= 0)[0]

        if len(x_filtered_indices) == 0:
            print("Error: No ground candidate points found after excluding positive X-coordinates.")
            pcd_display = o3d.geometry.PointCloud()
            pcd_display.points = pcd_original.points
            pcd_display.paint_uniform_color([0.5, 0.5, 0.5]) # Gray
            origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
            origin_sphere.paint_uniform_color([1, 1, 0]) # Yellow
            return pcd_display, origin_sphere, None

        ground_points = ground_points[x_filtered_indices]
        ground_indices = ground_indices[x_filtered_indices] # Update original indices as well
    
    # 3. Find the closest line of points by angular division using 3D Euclidean distance
    num_angular_bins = int(360 / angular_resolution)
    min_dist_per_bin = np.full(num_angular_bins, np.inf)
    closest_point_idx_per_bin = np.full(num_angular_bins, -1, dtype=int)

    # Calculate horizontal angles (azimuth) and 3D distances for all ground points
    angles = np.arctan2(ground_points[:, 1], ground_points[:, 0])
    bin_indices = ((angles + np.pi) / (2 * np.pi) * (num_angular_bins - 1)).astype(int)
    
    # *** Core Logic Change: Using full 3D Euclidean distance ***
    distances_3d = np.linalg.norm(ground_points, axis=1)

    # Find the closest points in each angular bin
    for i in range(len(ground_points)):
        bin_idx = bin_indices[i]
        dist = distances_3d[i]
        if dist < min_dist_per_bin[bin_idx]:
            min_dist_per_bin[bin_idx] = dist
            closest_point_idx_per_bin[bin_idx] = i
    
    # Use all valid bins
    valid_bins = [bin_idx for bin_idx in range(num_angular_bins) if closest_point_idx_per_bin[bin_idx] != -1]
    
    # Filter out bins that didn't have any points
    valid_indices_in_ground = closest_point_idx_per_bin[valid_bins]

    if len(valid_indices_in_ground) == 0:
        print("Error: Could not determine the closest line of points from the ground candidates.")
        # Visualize ground candidates for debugging
        pcd_display = o3d.geometry.PointCloud()
        pcd_display.points = pcd_original.points
        pcd_display.paint_uniform_color([0.5, 0.5, 0.5]) # Gray
        origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        origin_sphere.paint_uniform_color([1, 1, 0]) # Yellow
        return pcd_display, origin_sphere, None

    # 4. Prepare for visualization and saving

    # Get the points for the closest line
    closest_line_points = ground_points[valid_indices_in_ground]

    # Create a display point cloud from the original, and color it
    pcd_display = o3d.geometry.PointCloud()
    pcd_display.points = pcd_original.points
    
    # Initialize all points to gray
    colors = np.full(np.asarray(pcd_display.points).shape, [0.5, 0.5, 0.5])

    # Get the original indices of the closest line points
    original_indices_of_closest_line = ground_indices[valid_indices_in_ground]
    
    # Color the closest line points cyan within the display point cloud
    colors[original_indices_of_closest_line] = [0, 1, 1] # Cyan

    # Select a single purple point near y==0 (exclude x>0), and print its info
    all_points = np.asarray(pcd_display.points)
    x_vals = all_points[:, 0]
    y_vals = all_points[:, 1]

    # Candidates: |y| <= band and x <= 0
    y_zero_mask = (np.abs(y_vals) <= y_zero_band) & (x_vals <= 0.0)
    y_zero_indices = np.where(y_zero_mask)[0]

    # NEW: expose selected marker position via module-global
    global selected_marker_pos
    selected_marker_pos = None

    if y_zero_indices.size > 0:
        cand_points = all_points[y_zero_indices]
        dists_all = np.linalg.norm(cand_points, axis=1)

        # Use only positive distances
        pos_mask = dists_all > 0.0
        if (np.any(pos_mask)):
            cand_indices = y_zero_indices[pos_mask]
            dists_pos = dists_all[pos_mask]
            pts_pos = cand_points[pos_mask]

            # Pick the single closest candidate
            best_local = int(np.argmin(dists_pos))
            selected_idx = int(cand_indices[best_local])
            selected_point = pts_pos[best_local]
            selected_distance = float(dists_pos[best_local])
            selected_angle_deg = float(np.degrees(np.arctan2(selected_point[1], selected_point[0])))

            # Color only this one as purple
            colors[selected_idx] = [1.0, 0.0, 1.0]  # Purple

            # Print info
            selected_dict = {
                "x": float(selected_point[0]),
                "y": float(selected_point[1]),
                "z": float(selected_point[2]),
            }
            print(f"[y≈0 one-point] idx={selected_idx}, xyz={selected_dict}, dist={selected_distance:.4f} m, angle_deg={selected_angle_deg:.2f}")

            # NEW: remember marker position (for purple sphere)
            selected_marker_pos = selected_point.copy()
        else:
            print("[y≈0 one-point] no candidate with positive distance")
            selected_marker_pos = None
    else:
        print("[y≈0 one-point] no candidates within |y| band and x<=0")
        selected_marker_pos = None

    pcd_display.colors = o3d.utility.Vector3dVector(colors)

    # Highlight the origin with a yellow sphere
    origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    origin_sphere.paint_uniform_color([1, 1, 0]) # Yellow

    # 5. Save the resulting closest line to a new file
    # This part will be handled outside for batch processing
    
    print("-" * 30)
    print(f"Analysis Complete for current PCD")
    print(f"Method: 3D Euclidean Distance per {angular_resolution}-degree angle")
    print(f"Found {len(closest_line_points)} points for the closest line after filtering.")
    print("-" * 30)

    return pcd_display, origin_sphere, closest_line_points

def update_visualization(vis_instance):
    global current_pcd_idx, pcd_files, pcd_display_geometry, origin_sphere_geometry
    global selected_marker_geometry, selected_marker_pos, last_selected_marker_pos

    if not pcd_files:
        print("No PCD files to visualize.")
        return

    file_path = pcd_files[current_pcd_idx]
    print(f"Loading and analyzing: {os.path.basename(file_path)} ({current_pcd_idx + 1}/{len(pcd_files)})")

    try:
        pcd_original = o3d.io.read_point_cloud(file_path)
        if not pcd_original.has_points():
            print(f"Error: The point cloud {os.path.basename(file_path)} is empty. Skipping.")
            return

        # Call the analysis function
        display_pcd, origin_sphere, closest_line_points = find_closest_line_by_euclidean_distance(
            pcd_original,
            ground_z_min=-10.0,
            ground_z_max=-0.65,
            min_xy_distance_from_origin=1.0,
            xy_radius_threshold=2.85,
            exclude_positive_x=True,
            angular_resolution=0.85
        )

        if display_pcd is None: # Error occurred in analysis
            return

        # # Save the resulting closest line to a new file
        # if closest_line_points is not None:
        #     pcd_closest_line_only = o3d.geometry.PointCloud()
        #     pcd_closest_line_only.points = o3d.utility.Vector3dVector(closest_line_points)
        #     output_pcd_path = f"output_euclidean_closest_line_{os.path.basename(file_path).replace('.pcd', '')}.pcd"
        #     o3d.io.write_point_cloud(output_pcd_path, pcd_closest_line_only)
        #     print(f"Result PCD saved to: {output_pcd_path}")

        # Update visualization
        if pcd_display_geometry is None:
            pcd_display_geometry = o3d.geometry.PointCloud()
            pcd_display_geometry.points = o3d.utility.Vector3dVector(np.asarray(display_pcd.points))
            pcd_display_geometry.colors = o3d.utility.Vector3dVector(np.asarray(display_pcd.colors))
            vis_instance.add_geometry(pcd_display_geometry)
        else:
            pcd_display_geometry.points = o3d.utility.Vector3dVector(np.asarray(display_pcd.points))
            pcd_display_geometry.colors = o3d.utility.Vector3dVector(np.asarray(display_pcd.colors))
            vis_instance.update_geometry(pcd_display_geometry)
        
        if origin_sphere_geometry is None:
            origin_sphere_geometry = origin_sphere
            vis_instance.add_geometry(origin_sphere_geometry)
        else:
            # Sphere doesn't change, but we update it for consistency if it was added/removed
            vis_instance.update_geometry(origin_sphere_geometry)

        # NEW: draw/update blue vertical line from origin to z = -0.771 and a blue point there
        global blue_depth_line_geometry, blue_marker_geometry, last_blue_marker_pos, BLUE_DEPTH_Z
        blue_line_pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, BLUE_DEPTH_Z]], dtype=np.float64)
        blue_line_edges = np.array([[0, 1]], dtype=np.int32)
        if blue_depth_line_geometry is None:
            blue_depth_line_geometry = o3d.geometry.LineSet()
            blue_depth_line_geometry.points = o3d.utility.Vector3dVector(blue_line_pts)
            blue_depth_line_geometry.lines = o3d.utility.Vector2iVector(blue_line_edges)
            blue_depth_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, 1.0]], dtype=np.float64))  # blue
            vis_instance.add_geometry(blue_depth_line_geometry)
        else:
            blue_depth_line_geometry.points = o3d.utility.Vector3dVector(blue_line_pts)
            blue_depth_line_geometry.lines = o3d.utility.Vector2iVector(blue_line_edges)
            blue_depth_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, 1.0]], dtype=np.float64))
            vis_instance.update_geometry(blue_depth_line_geometry)

        blue_point_pos = np.array([0.0, 0.0, BLUE_DEPTH_Z], dtype=np.float64)
        if blue_marker_geometry is None:
            blue_marker_geometry = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)  # same as purple
            blue_marker_geometry.paint_uniform_color([0.0, 0.0, 1.0])  # Blue
            blue_marker_geometry.translate(blue_point_pos.tolist())
            vis_instance.add_geometry(blue_marker_geometry)
            last_blue_marker_pos = blue_point_pos.copy()
        else:
            # Move existing blue sphere if needed
            delta_blue = (blue_point_pos - last_blue_marker_pos).tolist()
            blue_marker_geometry.translate(delta_blue, relative=True)
            vis_instance.update_geometry(blue_marker_geometry)
            last_blue_marker_pos = blue_point_pos.copy()

        # NEW: add/update a larger purple sphere at the selected point (if any)
        if selected_marker_pos is not None:
            if selected_marker_geometry is None:
                selected_marker_geometry = o3d.geometry.TriangleMesh.create_sphere(radius=0.06)  # bigger than origin
                selected_marker_geometry.paint_uniform_color([1.0, 0.0, 1.0])  # Purple
                selected_marker_geometry.translate(selected_marker_pos.tolist())
                vis_instance.add_geometry(selected_marker_geometry)
                last_selected_marker_pos = selected_marker_pos.copy()
            else:
                # Move existing sphere to new position
                delta = (selected_marker_pos - last_selected_marker_pos).tolist()
                selected_marker_geometry.translate(delta, relative=True)
                vis_instance.update_geometry(selected_marker_geometry)
                last_selected_marker_pos = selected_marker_pos.copy()

            # NEW: draw/update black x-axis segment whose length equals the green line (origin→purple) distance
            global x_axis_line_geometry
            dist_len = float(np.linalg.norm(selected_marker_pos))  # Euclidean distance
            x_axis_pts = np.array([[0.0, 0.0, 0.0], [-dist_len, 0.0, 0.0]], dtype=np.float64)
            x_axis_lines = np.array([[0, 1]], dtype=np.int32)
            if x_axis_line_geometry is None:
                x_axis_line_geometry = o3d.geometry.LineSet()
                x_axis_line_geometry.points = o3d.utility.Vector3dVector(x_axis_pts)
                x_axis_line_geometry.lines = o3d.utility.Vector2iVector(x_axis_lines)
                x_axis_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, 0.0]], dtype=np.float64))  # black
                vis_instance.add_geometry(x_axis_line_geometry)
            else:
                x_axis_line_geometry.points = o3d.utility.Vector3dVector(x_axis_pts)
                x_axis_line_geometry.lines = o3d.utility.Vector2iVector(x_axis_lines)
                x_axis_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, 0.0]], dtype=np.float64))
                vis_instance.update_geometry(x_axis_line_geometry)

            # NEW: draw/update purple line from purple point to (0,0,z_purple) at constant z
            global vertical_purple_line_geometry
            z_end = float(selected_marker_pos[2])
            purple_line_pts = np.array([selected_marker_pos, [0.0, 0.0, z_end]], dtype=np.float64)
            purple_line_edges = np.array([[0, 1]], dtype=np.int32)
            if vertical_purple_line_geometry is None:
                vertical_purple_line_geometry = o3d.geometry.LineSet()
                vertical_purple_line_geometry.points = o3d.utility.Vector3dVector(purple_line_pts)
                vertical_purple_line_geometry.lines = o3d.utility.Vector2iVector(purple_line_edges)
                vertical_purple_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 1.0]], dtype=np.float64))  # purple
                vis_instance.add_geometry(vertical_purple_line_geometry)
            else:
                vertical_purple_line_geometry.points = o3d.utility.Vector3dVector(purple_line_pts)
                vertical_purple_line_geometry.lines = o3d.utility.Vector2iVector(purple_line_edges)
                vertical_purple_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 1.0]], dtype=np.float64))
                vis_instance.update_geometry(vertical_purple_line_geometry)

            # NEW: draw/update light-green line from blue point (0,0,BLUE_DEPTH_Z) to the selected purple point
            global blue_to_purple_line_geometry
            blue_point_pos = np.array([0.0, 0.0, BLUE_DEPTH_Z], dtype=np.float64)
            # Extend beyond the purple point along the same direction
            dir_vec = (selected_marker_pos - blue_point_pos).astype(np.float64)
            norm = float(np.linalg.norm(dir_vec))
            if norm > 1e-8:
                extended_end = blue_point_pos + GREEN_EXTEND_FACTOR * dir_vec
            else:
                extended_end = selected_marker_pos.astype(np.float64)
            green_pts = np.array([blue_point_pos, extended_end], dtype=np.float64)
            green_edges = np.array([[0, 1]], dtype=np.int32)
            if blue_to_purple_line_geometry is None:
                blue_to_purple_line_geometry = o3d.geometry.LineSet()
                blue_to_purple_line_geometry.points = o3d.utility.Vector3dVector(green_pts)
                blue_to_purple_line_geometry.lines = o3d.utility.Vector2iVector(green_edges)
                blue_to_purple_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[0.6, 1.0, 0.2]], dtype=np.float64))  # light green
                vis_instance.add_geometry(blue_to_purple_line_geometry)
            else:
                blue_to_purple_line_geometry.points = o3d.utility.Vector3dVector(green_pts)
                blue_to_purple_line_geometry.lines = o3d.utility.Vector2iVector(green_edges)
                blue_to_purple_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[0.6, 1.0, 0.2]], dtype=np.float64))
                vis_instance.update_geometry(blue_to_purple_line_geometry)

            # NEW: draw/update light-green line from origin to purple point
            global distance_line_geometry
            dist_pts = np.array([[0.0, 0.0, 0.0], selected_marker_pos], dtype=np.float64)
            dist_lines = np.array([[0, 1]], dtype=np.int32)
            if distance_line_geometry is None:
                distance_line_geometry = o3d.geometry.LineSet()
                distance_line_geometry.points = o3d.utility.Vector3dVector(dist_pts)
                distance_line_geometry.lines = o3d.utility.Vector2iVector(dist_lines)
                distance_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 0.0]], dtype=np.float64))  # Red
                vis_instance.add_geometry(distance_line_geometry)
            else:
                distance_line_geometry.points = o3d.utility.Vector3dVector(dist_pts)
                distance_line_geometry.lines = o3d.utility.Vector2iVector(dist_lines)
                distance_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 0.0]], dtype=np.float64))
                vis_instance.update_geometry(distance_line_geometry)

            # ===================== Metrics & Diagram Print =====================
            # Define variables by color:
            # a: red (origin → purple)
            # b: magenta (base length on XY from origin to purple projected on z=const)
            # f+g: blue total target height (configured)
            # f: lower blue segment = |BLUE_DEPTH_Z|
            # g: upper yellow segment = (f+g) - f

            x_purple, y_purple, z_purple = float(selected_marker_pos[0]), float(selected_marker_pos[1]), float(selected_marker_pos[2])
            a_len = float(np.linalg.norm(selected_marker_pos))
            b_len = float(np.hypot(x_purple, y_purple))
            blue_point_pos = np.array([0.0, 0.0, BLUE_DEPTH_Z], dtype=np.float64)

            # Heights (dynamic)
            f_lower = float(abs(BLUE_DEPTH_Z))                 # f
            g_upper = float(abs(z_purple - BLUE_DEPTH_Z))      # g = |z_purple - BLUE_DEPTH_Z|
            fg_total = float(f_lower + g_upper)                # f+g

            # Draw/update g yellow vertical line at x=y=0
            global yellow_g_line_geometry
            g_start = np.array([0.0, 0.0, BLUE_DEPTH_Z], dtype=np.float64)
            g_end = np.array([0.0, 0.0, z_purple], dtype=np.float64)
            g_pts = np.stack([g_start, g_end], axis=0)
            g_edges = np.array([[0, 1]], dtype=np.int32)
            if yellow_g_line_geometry is None:
                yellow_g_line_geometry = o3d.geometry.LineSet()
                yellow_g_line_geometry.points = o3d.utility.Vector3dVector(g_pts)
                yellow_g_line_geometry.lines = o3d.utility.Vector2iVector(g_edges)
                yellow_g_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[1.0, 1.0, 0.0]], dtype=np.float64))  # yellow
                vis_instance.add_geometry(yellow_g_line_geometry)
            else:
                yellow_g_line_geometry.points = o3d.utility.Vector3dVector(g_pts)
                yellow_g_line_geometry.lines = o3d.utility.Vector2iVector(g_edges)
                yellow_g_line_geometry.colors = o3d.utility.Vector3dVector(np.array([[1.0, 1.0, 0.0]], dtype=np.float64))
                vis_instance.update_geometry(yellow_g_line_geometry)

            # Compute angles only (remove all arc drawings)
            # a = vector OP (origin->purple), b = its XY projection, c = vector from blue point (0,0,BLUE_DEPTH_Z) to purple
            c_vec = (selected_marker_pos - blue_point_pos).astype(np.float64)
            c_len = float(np.linalg.norm(c_vec))

            eps = 1e-9
            if a_len > eps and b_len > eps:
                # d = angle(a,b) = acos( b / |a| ) = atan2(f+g, b)
                d_angle = float(np.degrees(np.arccos(np.clip(b_len / max(a_len, eps), -1.0, 1.0))))
            else:
                d_angle = 0.0

            if c_len > eps and b_len > eps:
                # e = angle(c,b) = acos( b / |c| ) = atan2(g, b)
                e_angle = float(np.degrees(np.arccos(np.clip(b_len / max(c_len, eps), -1.0, 1.0))))
            else:
                e_angle = 0.0

            x_angle = float(d_angle - e_angle)

            # Remove any previously drawn arcs
            global arc_d_geometry, arc_e_geometry, arc_x_geometry
            if arc_d_geometry is not None:
                vis_instance.remove_geometry(arc_d_geometry, reset_bounding_box=False)
                arc_d_geometry = None
            if arc_e_geometry is not None:
                vis_instance.remove_geometry(arc_e_geometry, reset_bounding_box=False)
                arc_e_geometry = None
            if arc_x_geometry is not None:
                vis_instance.remove_geometry(arc_x_geometry, reset_bounding_box=False)
                arc_x_geometry = None

            # NEW: draw/update circles on a plane tilted by c's slope (centered at blue point)
            global c_circles_geometry, c_circles_prev_geometry, c_circles_pcd_geometry, NUM_C_RADII, CIRCLE_SEGS, SKIP_FAR_COUNT, XY_MIN_SEPARATION

            if c_len > eps:
                # Basis aligned to purple azimuth
                u_b_xy = np.array([x_purple, y_purple, 0.0], dtype=np.float64)
                if np.linalg.norm(u_b_xy) <= eps:
                    u_b_xy = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                else:
                    u_b_xy /= np.linalg.norm(u_b_xy)
                z_hat = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                u_tan = np.cross(z_hat, u_b_xy)  # sideways, in XY

                # Tilt to match c's slope along u_b direction
                # slope = dz / dr along u_b
                slope_c = float((z_purple - BLUE_DEPTH_Z) / max(b_len, eps))
                u_elev = u_b_xy + slope_c * z_hat
                u_elev /= max(np.linalg.norm(u_elev), eps)  # normalize

                # Radii along c, skip near region
                radii_along_c = build_radii_along_c_vals(c_len)

                center = np.array([0.0, 0.0, BLUE_DEPTH_Z], dtype=np.float64)  # keep center at blue point
                all_pts, all_lines, all_colors = [], [], []
                pcd_pts_list = []  # X<=0 포인트만 모아 PCD 생성
                # Build XY KD-tree of original PCD (z=0로 임베딩하여 XY거리 사용)
                orig_xy_kdtree = None
                _kd_pcd_xy = None
                try:
                    # 항상 원본 PCD에서 KDTree 구성 (디스플레이 버퍼와 분리)
                    orig_pts = np.asarray(pcd_original.points)
                    if orig_pts.size > 0:
                        orig_xy = np.column_stack([orig_pts[:, 0], orig_pts[:, 1],
                                                   np.zeros((orig_pts.shape[0],), dtype=np.float64)])
                        _kd_pcd_xy = o3d.geometry.PointCloud()
                        _kd_pcd_xy.points = o3d.utility.Vector3dVector(orig_xy.astype(np.float64))
                        orig_xy_kdtree = o3d.geometry.KDTreeFlann(_kd_pcd_xy)
                except Exception as _e:
                    print(f"KDTree build failed: {_e}")

                base_idx = 0
                ts = np.linspace(0.0, 2.0 * np.pi, CIRCLE_SEGS, dtype=np.float64)
                cos_t = np.cos(ts)[:, None]
                sin_t = np.sin(ts)[:, None]

                for r in radii_along_c:
                    # 1) 보라색점을 지나는 원(r == c_len)은 그리지 않음
                    if abs(r - c_len) <= max(1e-9, 1e-6 * c_len):
                        continue

                    # Circle in tilted plane spanned by (u_elev, u_tan) -> BLACK (current method)
                    ring = center + r * (cos_t * u_elev + sin_t * u_tan)

                    # 2) X>0 제거: X<=0인 점만 허용 (라인)
                    all_pts.append(ring)
                    idxs = np.arange(CIRCLE_SEGS, dtype=np.int32)
                    nxts = (idxs + 1) % CIRCLE_SEGS
                    valid = (ring[:, 0] <= 0.0)
                    edge_mask = valid[idxs] & valid[nxts]
                    if np.any(edge_mask):
                        lines = np.column_stack(
                            [base_idx + idxs[edge_mask], base_idx + nxts[edge_mask]]
                        ).astype(np.int32)
                        all_lines.append(lines)
                        all_colors.append(np.tile([0.0, 0.0, 0.0], (lines.shape[0], 1)))  # black
                    # 3) PCD용 포인트: X<=0 AND 기존 PCD와 XY거리 >= XY_MIN_SEPARATION
                    ring_valid_pts = ring[valid]
                    if ring_valid_pts.size > 0:
                        if orig_xy_kdtree is not None:
                            kept = []
                            for p in ring_valid_pts:
                                q = np.array([p[0], p[1], 0.0], dtype=np.float64)
                                try:
                                    k, _idx, _d2 = orig_xy_kdtree.search_radius_vector_3d(q, float(XY_MIN_SEPARATION))
                                    if k == 0:
                                        kept.append(p)
                                except Exception:
                                    kept.append(p)  # 안전하게 유지
                            if kept:
                                pcd_pts_list.append(np.asarray(kept, dtype=np.float64))
                        else:
                            pcd_pts_list.append(ring_valid_pts)
                    base_idx += CIRCLE_SEGS

                all_pts_np = np.vstack(all_pts) if all_pts else np.zeros((0, 3), dtype=np.float64)
                all_lines_np = np.vstack(all_lines) if all_lines else np.zeros((0, 2), dtype=np.int32)
                all_colors_np = np.vstack(all_colors) if all_colors else np.zeros((0, 3), dtype=np.float64)
                if c_circles_geometry is None:
                    c_circles_geometry = o3d.geometry.LineSet()
                    c_circles_geometry.points = o3d.utility.Vector3dVector(all_pts_np)
                    c_circles_geometry.lines = o3d.utility.Vector2iVector(all_lines_np)
                    c_circles_geometry.colors = o3d.utility.Vector3dVector(all_colors_np)
                    vis_instance.add_geometry(c_circles_geometry)
                else:
                    c_circles_geometry.points = o3d.utility.Vector3dVector(all_pts_np)
                    c_circles_geometry.lines = o3d.utility.Vector2iVector(all_lines_np)
                    c_circles_geometry.colors = o3d.utility.Vector3dVector(all_colors_np)
                    vis_instance.update_geometry(c_circles_geometry)

                # 원 포인트들을 PCD로 추가/업데이트
                pcd_pts_np = np.vstack(pcd_pts_list) if pcd_pts_list else np.zeros((0, 3), dtype=np.float64)
                if pcd_pts_np.shape[0] > 0:
                    if c_circles_pcd_geometry is None:
                        c_circles_pcd_geometry = o3d.geometry.PointCloud()
                        c_circles_pcd_geometry.points = o3d.utility.Vector3dVector(pcd_pts_np)
                        c_circles_pcd_geometry.colors = o3d.utility.Vector3dVector(
                            np.tile(np.array([[0.0, 0.0, 0.0]], dtype=np.float64), (pcd_pts_np.shape[0], 1))
                        )  # black
                        vis_instance.add_geometry(c_circles_pcd_geometry)
                    else:
                        c_circles_pcd_geometry.points = o3d.utility.Vector3dVector(pcd_pts_np)
                        c_circles_pcd_geometry.colors = o3d.utility.Vector3dVector(
                            np.tile(np.array([[0.0, 0.0, 0.0]], dtype=np.float64), (pcd_pts_np.shape[0], 1))
                        )
                        vis_instance.update_geometry(c_circles_pcd_geometry)
                else:
                    if c_circles_pcd_geometry is not None:
                        vis_instance.remove_geometry(c_circles_pcd_geometry, reset_bounding_box=False)
                        c_circles_pcd_geometry = None

                # ALSO draw previous method (RED): ground plane circles -> 요청: 주석처리(비활성)
                if DRAW_PREV_CIRCLES:
                    all_pts_prev, all_lines_prev, all_colors_prev = [], [], []
                    base_idx_prev = 0
                    for r in radii_along_c:
                        ring_prev = center + r * (cos_t * u_b_xy + sin_t * u_tan)  # ground plane
                        all_pts_prev.append(ring_prev)
                        idxs_prev = np.arange(CIRCLE_SEGS, dtype=np.int32)
                        lines_prev = np.column_stack([base_idx_prev + idxs_prev, base_idx_prev + (idxs_prev + 1) % CIRCLE_SEGS]).astype(np.int32)
                        all_lines_prev.append(lines_prev)
                        all_colors_prev.append(np.tile([1.0, 0.0, 0.0], (lines_prev.shape[0], 1)))  # red
                        base_idx_prev += CIRCLE_SEGS

                    all_pts_prev_np = np.vstack(all_pts_prev) if all_pts_prev else np.zeros((0, 3), dtype=np.float64)
                    all_lines_prev_np = np.vstack(all_lines_prev) if all_lines_prev else np.zeros((0, 2), dtype=np.int32)
                    all_colors_prev_np = np.vstack(all_colors_prev) if all_colors_prev else np.zeros((0, 3), dtype=np.float64)
                    if c_circles_prev_geometry is None:
                        c_circles_prev_geometry = o3d.geometry.LineSet()
                        c_circles_prev_geometry.points = o3d.utility.Vector3dVector(all_pts_prev_np)
                        c_circles_prev_geometry.lines = o3d.utility.Vector2iVector(all_lines_prev_np)
                        c_circles_prev_geometry.colors = o3d.utility.Vector3dVector(all_colors_prev_np)
                        vis_instance.add_geometry(c_circles_prev_geometry)
                    else:
                        c_circles_prev_geometry.points = o3d.utility.Vector3dVector(all_pts_prev_np)
                        c_circles_prev_geometry.lines = o3d.utility.Vector2iVector(all_lines_prev_np)
                        c_circles_prev_geometry.colors = o3d.utility.Vector3dVector(all_colors_prev_np)
                        vis_instance.update_geometry(c_circles_prev_geometry)
                else:
                    if c_circles_prev_geometry is not None:
                        vis_instance.remove_geometry(c_circles_prev_geometry, reset_bounding_box=False)
                        c_circles_prev_geometry = None

            else:
                if c_circles_geometry is not None:
                    vis_instance.remove_geometry(c_circles_geometry, reset_bounding_box=False)
                    c_circles_geometry = None
                if c_circles_prev_geometry is not None:
                    vis_instance.remove_geometry(c_circles_prev_geometry, reset_bounding_box=False)
                    c_circles_prev_geometry = None
                if c_circles_pcd_geometry is not None:
                    vis_instance.remove_geometry(c_circles_pcd_geometry, reset_bounding_box=False)
                    c_circles_pcd_geometry = None

            print("================ Diagram (angles & height) ================")
            print(f"a (origin→purple, red)       : {a_len:.3f} m")
            print(f"b (base, magenta)            : {b_len:.3f} m")
            print(f"f+g (blue total)             : {fg_total:.3f} m  (= |BLUE_DEPTH_Z| + |z_purple-BLUE_DEPTH_Z|)")
            print(f"g (yellow)                    : {g_upper:.3f} m  (= |z_purple-BLUE_DEPTH_Z|)")
            print(f"f (blue)                      : {f_lower:.3f} m  (= |BLUE_DEPTH_Z|)")
            print(f"d (red/base)                 : {d_angle:.2f} deg   (= angle(a,b)=acos(b/|a|)=atan2(f+g, b))")
            print(f"e (green/base)               : {e_angle:.2f} deg   (= angle(c,b)=acos(b/|c|)=atan2(g, b))")
            print(f"x (red−green)                : {x_angle:.2f} deg   (= d - e)")
            print(f"[fit] points used            : {len(closest_line_points)}")
            print("----------------------------------------------------------")
            # ==================================================================
        else:
            # If no selection, remove lines if they exist
            if x_axis_line_geometry is not None:
                vis_instance.remove_geometry(x_axis_line_geometry, reset_bounding_box=False)
                x_axis_line_geometry = None
            if distance_line_geometry is not None:
                vis_instance.remove_geometry(distance_line_geometry, reset_bounding_box=False)
                distance_line_geometry = None
            if vertical_purple_line_geometry is not None:
                vis_instance.remove_geometry(vertical_purple_line_geometry, reset_bounding_box=False)
                vertical_purple_line_geometry = None
            if blue_to_purple_line_geometry is not None:
                vis_instance.remove_geometry(blue_to_purple_line_geometry, reset_bounding_box=False)
                blue_to_purple_line_geometry = None
            # remove angle arcs
            if arc_d_geometry is not None:
                vis_instance.remove_geometry(arc_d_geometry, reset_bounding_box=False)
                arc_d_geometry = None
            if arc_e_geometry is not None:
                vis_instance.remove_geometry(arc_e_geometry, reset_bounding_box=False)
                arc_e_geometry = None
            if arc_x_geometry is not None:
                vis_instance.remove_geometry(arc_x_geometry, reset_bounding_box=False)
                arc_x_geometry = None
            # remove c circles
            if c_circles_geometry is not None:
                vis_instance.remove_geometry(c_circles_geometry, reset_bounding_box=False)
                c_circles_geometry = None
            if c_circles_prev_geometry is not None:
                vis_instance.remove_geometry(c_circles_prev_geometry, reset_bounding_box=False)
                c_circles_prev_geometry = None
            if c_circles_pcd_geometry is not None:
                vis_instance.remove_geometry(c_circles_pcd_geometry, reset_bounding_box=False)
                c_circles_pcd_geometry = None

        vis_instance.update_renderer()
        vis_instance.poll_events()

    except Exception as e:
        print(f"An error occurred while processing {os.path.basename(file_path)}: {e}")

def next_pcd(vis_instance):
    global current_pcd_idx, pcd_files
    if not pcd_files:
        return
    current_pcd_idx = (current_pcd_idx + 1) % len(pcd_files)
    update_visualization(vis_instance)

def prev_pcd(vis_instance):
    global current_pcd_idx, pcd_files
    if not pcd_files:
        return
    current_pcd_idx = (current_pcd_idx - 1 + len(pcd_files)) % len(pcd_files)
    update_visualization(vis_instance)

# Build radii along c with near-center bias
def build_radii_along_c_vals(c_len: float) -> np.ndarray:
    # Uses NUM_C_RADII rings from near-zero up to c_len (excluding zero),
    # optional biasing by RADIUS_DISTRIBUTION.
    if c_len <= 0 or NUM_C_RADII <= 0:
        return np.zeros((0,), dtype=np.float64)
    t = np.linspace(0.0, 1.0, NUM_C_RADII + 1, dtype=np.float64)[1:]  # drop zero
    if RADIUS_DISTRIBUTION == "near":
        # 중심(blue) 근처에 촘촘
        if NEAR_BIAS != 1.0:
            t = t ** NEAR_BIAS
    elif RADIUS_DISTRIBUTION == "cosine":
        # 양 끝(0과 |c|)에서 촘촘: t' = (1 - cos(pi t)) / 2
        t = (1.0 - np.cos(np.pi * t)) * 0.5
    # else 'uniform': 그대로 사용
    radii = t * c_len
    if MIN_FIRST_RADIUS > 0.0:
        radii = radii[radii >= MIN_FIRST_RADIUS]
    # 먼쪽(보라색점 방향)에서 N개 제거
    if SKIP_FAR_COUNT > 0:
        keep = max(len(radii) - int(SKIP_FAR_COUNT), 0)
        radii = radii[:keep]
    return radii

# === Ground grid (z=0) helpers ===
def build_ground_grid(size_x=20.0, size_y=20.0, step=1.0, z=0.0, color=(0.6, 0.6, 0.6)) -> o3d.geometry.LineSet:
    xs = np.arange(-size_x, size_x + 1e-9, step, dtype=np.float64)
    ys = np.arange(-size_y, size_y + 1e-9, step, dtype=np.float64)

    pts = []
    lines = []
    colors = []

    # vertical lines (constant x)
    idx = 0
    for x in xs:
        pts.append([x, -size_y, z])
        pts.append([x,  size_y, z])
        lines.append([idx, idx + 1]); idx += 2
        colors.append(list(color))
    # horizontal lines (constant y)
    for y in ys:
        pts.append([-size_x, y, z])
        pts.append([ size_x, y, z])
        lines.append([idx, idx + 1]); idx += 2
        colors.append(list(color))

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
    ls.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    ls.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float64))
    return ls

def toggle_ground_plane(vis_instance):
    global ground_plane_geometry, GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_STEP
    try:
        if ground_plane_geometry is None:
            ground_plane_geometry = build_ground_grid(GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_STEP, z=BLUE_DEPTH_Z, color=(0.6, 0.6, 0.6))
            vis_instance.add_geometry(ground_plane_geometry)
        else:
            vis_instance.remove_geometry(ground_plane_geometry, reset_bounding_box=False)
            ground_plane_geometry = None
        vis_instance.update_renderer()
    except Exception as e:
        print(f"toggle_ground_plane error: {e}")

def run_batch_processing(input_path, output_base_path):
    """
    배치 모드로 모든 PCD 파일을 처리하고, 각 파일의 closest_line_points를 개별 JSON으로 저장.
    """
    if os.path.isdir(input_path):
        pcd_files = sorted([os.path.join(input_path, f) for f in os.listdir(input_path) if f.endswith('.pcd')])
        if not pcd_files:
            print(f"Error: No .pcd files found in directory {input_path}")
            return
    elif os.path.isfile(input_path) and input_path.endswith('.pcd'):
        pcd_files = [input_path]
    else:
        print(f"Error: Invalid input path. Must be a .pcd file or a directory containing .pcd files: {input_path}")
        return

    # 출력 폴더 생성
    output_dir = os.path.join(output_base_path, "closest_line_points")
    os.makedirs(output_dir, exist_ok=True)

    print(f"Starting batch processing for {len(pcd_files)} PCD files...")

    for file_path in pcd_files:
        filename = os.path.basename(file_path).replace('.pcd', '')
        print(f"Processing: {filename}")

        try:
            pcd_original = o3d.io.read_point_cloud(file_path)
            if not pcd_original.has_points():
                print(f"Skipping empty PCD: {filename}")
                continue

            # 분석 함수 호출
            _, _, closest_line_points = find_closest_line_by_euclidean_distance(
                pcd_original,
                ground_z_min=-10.0,
                ground_z_max=-0.85,
                min_xy_distance_from_origin=1.0,
                xy_radius_threshold=3.0,
                exclude_positive_x=True,
                angular_resolution=0.1
            )

            if closest_line_points is not None and len(closest_line_points) > 0:
                # 각 파일별 딕셔너리 생성 및 JSON 저장
                file_data = {
                    "x": closest_line_points[:, 0].tolist(),
                    "y": closest_line_points[:, 1].tolist(),
                    "z": closest_line_points[:, 2].tolist()
                }
                
                # 개별 JSON 파일로 저장
                output_json_path = os.path.join(output_dir, f"{filename}.json")
                with open(output_json_path, 'w') as f:
                    json.dump(file_data, f, indent=4)
                
                print(f"  -> Saved {len(closest_line_points)} points to {output_json_path}")
            else:
                print(f"  -> No closest line points found for {filename}")

        except Exception as e:
            print(f"Error processing {filename}: {e}")

    print(f"Batch processing complete. Individual JSON files saved in: {output_dir}")

def run_interactive_viewer(input_path):
    global vis, pcd_files, current_pcd_idx

    if os.path.isdir(input_path):
        pcd_files = sorted([os.path.join(input_path, f) for f in os.listdir(input_path) if f.endswith('.pcd')])
        if not pcd_files:
            print(f"Error: No .pcd files found in directory {input_path}")
            return
    elif os.path.isfile(input_path) and input_path.endswith('.pcd'):
        pcd_files = [input_path]
    else:
        print(f"Error: Invalid input path. Must be a .pcd file or a directory containing .pcd files: {input_path}")
        return

    print(f"Found {len(pcd_files)} PCD files.")
    print("Use 'n' for next, 'p' for previous. Press 'Q' or 'Esc' to quit.")
    print("Press 'G' to toggle ground grid (z=BLUE_DEPTH_Z).")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Closest Line by 3D Euclidean Distance (Interactive)", width=1024, height=768)

    # NEW: make all points larger for visibility
    render_opt = vis.get_render_option()
    render_opt.point_size = 5.0  # adjust as needed

    # Register key callbacks
    vis.register_key_callback(ord('N'), next_pcd)
    vis.register_key_callback(ord('P'), prev_pcd)
    vis.register_key_callback(ord('G'), toggle_ground_plane)  # toggle ground grid
    
    # Initial visualization
    update_visualization(vis)

    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find the closest line of points in a PCD file or a folder of PCD files.")
    parser.add_argument("input_path", type=str, 
                        help="Path to a single .pcd file or a directory containing .pcd files.")
    parser.add_argument("--auto", action="store_true", 
                        help="Run in batch mode to process all files automatically and save results.")
    
    args = parser.parse_args()

    if args.auto:
        # 배치 모드: ncdb-cls-sample\synced_data를 기본 출력 경로로 사용
        output_base_path = r"C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\ncdb-cls-sample\synced_data"
        run_batch_processing(args.input_path, output_base_path)
    else:
        # 인터랙티브 모드
        run_interactive_viewer(args.input_path)
