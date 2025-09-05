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

# NEW: light-green line from blue point to the purple line at z=z_purple
blue_to_purple_line_geometry = None
# 선 연장 배율(>1이면 보라색 점을 넘어 연장)
GREEN_EXTEND_FACTOR = 10.0

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
            pcd_display_geometry = display_pcd
            vis_instance.add_geometry(pcd_display_geometry)
        else:
            pcd_display_geometry.points = display_pcd.points
            pcd_display_geometry.colors = display_pcd.colors
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

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Closest Line by 3D Euclidean Distance (Interactive)", width=1024, height=768)

    # NEW: make all points larger for visibility
    render_opt = vis.get_render_option()
    render_opt.point_size = 5.0  # adjust as needed

    # Register key callbacks
    vis.register_key_callback(ord('N'), next_pcd)
    vis.register_key_callback(ord('P'), prev_pcd)
    
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
