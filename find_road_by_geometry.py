# Note: The user wants to find the nearest road point using only XYZ coordinates.
# This script identifies the ground plane by filtering points based on their Z-coordinate,
# then finds the point on that plane closest to the LiDAR origin.
# The filtering criteria have been refined based on user feedback to include XY proximity and Y-axis range.
# The initial camera view for visualization is now set to a front-facing perspective.
# A new visualization element is added: a circle representing the minimum radius, centered at (0, 0, closest_point.z).

import open3d as o3d
import numpy as np
import os

def find_nearest_road_point_by_height_and_xy_proximity(
    file_path,
    ground_z_min=-3.0,  # Wider range for initial ground candidates
    ground_z_max=0.0,
    min_xy_distance_from_origin=2.0, # Minimum horizontal distance from origin
    xy_radius_threshold=10.0,  # Max horizontal distance from origin for road candidates
    y_min=None, # New: Minimum Y-coordinate for road candidates
    y_max=None  # New: Maximum Y-coordinate for road candidates
):
    """
    Loads a PCD, filters initial ground points by Z-height, then refines by XY proximity and Y-axis range,
    finds the closest one to the origin, and visualizes the result.
    """
    if not os.path.exists(file_path):
        print(f"Error: File not found at {file_path}")
        return

    try:
        # 1. Load the point cloud
        pcd_original = o3d.io.read_point_cloud(file_path)
        if not pcd_original.has_points():
            print("Error: The point cloud is empty.")
            return
            
        points = np.asarray(pcd_original.points)

        # Initialize colors for all points to gray
        colors = np.full(points.shape, [0.5, 0.5, 0.5]) # Default gray

        # 2. First-pass filtering for initial ground candidates based on Z-coordinate
        initial_ground_indices = np.where(
            (points[:, 2] > ground_z_min) & (points[:, 2] < ground_z_max)
        )[0]
        
        if len(initial_ground_indices) == 0:
            print(f"Error: No initial ground candidates found in Z-range ({ground_z_min} to {ground_z_max}).")
            print("Please adjust the ground_z_min and ground_z_max parameters.")
            # Visualize original pcd anyway for debugging
            pcd_original.colors = o3d.utility.Vector3dVector(colors)
            o3d.visualization.draw_geometries([pcd_original], window_name="Original PCD (No Initial Ground Found)")
            return

        # Apply light blue color to initial ground candidates
        colors[initial_ground_indices] = [0.7, 0.7, 0.9] # Light blue

        # 3. Second-pass filtering for final road candidates based on XY proximity and Y-axis range
        initial_ground_points = points[initial_ground_indices]
        xy_distances_sq = np.sum(initial_ground_points[:, :2]**2, axis=1) # Only X and Y
        
        # Start with XY distance conditions
        filter_conditions = (
            (xy_distances_sq > min_xy_distance_from_origin**2) & 
            (xy_distances_sq < xy_radius_threshold**2)
        )
        
        # Add Y-axis filtering if parameters are provided
        if y_min is not None and y_max is not None:
            filter_conditions = (
                filter_conditions & 
                ((initial_ground_points[:, 1] > y_min) & 
                 (initial_ground_points[:, 1] < y_max))
            )

        final_road_indices_in_initial = np.where(filter_conditions)[0]

        if len(final_road_indices_in_initial) == 0:
            print(f"Error: No final road candidates found with current filtering parameters.")
            print(f"Z-range: ({ground_z_min} to {ground_z_max}), XY-range: ({min_xy_distance_from_origin}m to {xy_radius_threshold}m)")
            if y_min is not None and y_max is not None:
                print(f"Y-range: ({y_min}m to {y_max}m)")
            print("Please adjust the filtering parameters.")
            # Visualize initial ground candidates for debugging
            pcd_original.colors = o3d.utility.Vector3dVector(colors)
            o3d.visualization.draw_geometries([pcd_original], window_name="Initial Ground Candidates (No Final Road Found)")
            return

        # Apply dark blue color to final road candidates
        original_indices_of_final_road = initial_ground_indices[final_road_indices_in_initial]
        colors[original_indices_of_final_road] = [0, 0, 1] # Dark blue

        final_road_points = points[original_indices_of_final_road]
        
        # 4. Find the closest final road point to the origin (0, 0, 0)
        distances_sq_to_origin = np.sum(final_road_points**2, axis=1) # Full 3D distance
        min_dist_idx = np.argmin(distances_sq_to_origin)
        
        closest_point = final_road_points[min_dist_idx]
        min_radius = np.sqrt(distances_sq_to_origin[min_dist_idx])
        
        print("-" * 30)
        print(f"Analysis Results for {os.path.basename(file_path)}")
        print(f"Found {len(initial_ground_indices)} initial ground candidates (Z-filtered).")
        print(f"Found {len(final_road_points)} final road candidates (Z, XY, Y-filtered).")
        print(f"Closest road point coordinates: {closest_point}")
        print(f"Minimum radius from origin: {min_radius:.4f} meters")
        print("-" * 30)

        # 5. Prepare for visualization
        pcd_original.colors = o3d.utility.Vector3dVector(colors)
        
        # Highlight the closest point with a red sphere
        closest_point_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
        closest_point_sphere.translate(closest_point)
        closest_point_sphere.paint_uniform_color([1, 0, 0]) # Red

        # Highlight the origin with a yellow sphere
        origin_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        origin_sphere.paint_uniform_color([1, 1, 0]) # Yellow
        
        # Create a line to represent the minimum radius from origin to closest point
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([[0, 0, 0], closest_point]),
            lines=o3d.utility.Vector2iVector([[0, 1]]),
        )
        line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0]]) # Red line

        # New: Define the new radius center at (0, 0, closest_point.z)
        new_radius_center = np.array([0.0, 0.0, closest_point[2]])
        new_radius_center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
        new_radius_center_sphere.translate(new_radius_center)
        new_radius_center_sphere.paint_uniform_color([0, 1, 0]) # Green for new center

        # New: Create a circle at the new_radius_center with min_radius
        # This will be a series of points forming a circle
        num_points_circle = 100
        theta = np.linspace(0, 2 * np.pi, num_points_circle)
        circle_points_2d = np.array([min_radius * np.cos(theta), min_radius * np.sin(theta)]).T
        circle_points_3d = np.hstack((circle_points_2d, np.full((num_points_circle, 1), new_radius_center[2])))
        
        circle_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(circle_points_3d),
            lines=o3d.utility.Vector2iVector([[i, (i + 1) % num_points_circle] for i in range(num_points_circle)])
        )
        circle_line_set.colors = o3d.utility.Vector3dVector(np.full((num_points_circle, 3), [0, 1, 0])) # Green circle

        # New: Create multiple concentric circles by dividing radius into 10 segments
        num_radius_divisions = 10
        points_per_circle = 100
        concentric_circles = []
        
        for i in range(1, num_radius_divisions + 1):  # 1 to 10 (avoid radius=0)
            current_radius = (i / num_radius_divisions) * min_radius
            
            # Generate circle points
            theta = np.linspace(0, 2 * np.pi, points_per_circle)
            circle_x = current_radius * np.cos(theta)
            circle_y = current_radius * np.sin(theta)
            circle_z = np.full(points_per_circle, new_radius_center[2])  # Fixed Z at closest_point.z
            
            circle_points_3d = np.column_stack((circle_x, circle_y, circle_z))
            
            # Create LineSet for this circle
            circle_lines = [[j, (j + 1) % points_per_circle] for j in range(points_per_circle)]
            
            concentric_circle = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(circle_points_3d),
                lines=o3d.utility.Vector2iVector(circle_lines)
            )
            
            # Color gradient: inner circles (cyan) -> outer circles (magenta)
            color_ratio = i / num_radius_divisions
            color = [color_ratio, 1 - color_ratio, 1.0]  # Cyan to Magenta gradient
            concentric_circle.colors = o3d.utility.Vector3dVector(np.full((points_per_circle, 3), color))
            
            concentric_circles.append(concentric_circle)
        
        # Create a point cloud from all concentric circle points for potential PCD export
        all_circle_points = []
        for i in range(1, num_radius_divisions + 1):
            current_radius = (i / num_radius_divisions) * min_radius
            theta = np.linspace(0, 2 * np.pi, points_per_circle)
            circle_x = current_radius * np.cos(theta)
            circle_y = current_radius * np.sin(theta)
            circle_z = np.full(points_per_circle, new_radius_center[2])
            circle_points_3d = np.column_stack((circle_x, circle_y, circle_z))
            all_circle_points.append(circle_points_3d)
        
        all_circle_points = np.vstack(all_circle_points)
        synthetic_pcd = o3d.geometry.PointCloud()
        synthetic_pcd.points = o3d.utility.Vector3dVector(all_circle_points)
        
        # Color the synthetic points with rainbow gradient
        colors_synthetic = []
        for i in range(1, num_radius_divisions + 1):
            color_ratio = i / num_radius_divisions
            color = [color_ratio, 1 - color_ratio, 1.0]
            colors_synthetic.extend([color] * points_per_circle)
        synthetic_pcd.colors = o3d.utility.Vector3dVector(np.array(colors_synthetic))

        print(f"Generated {len(all_circle_points)} synthetic points in {num_radius_divisions} concentric circles")
        print(f"Each circle has {points_per_circle} points at radius increments of {min_radius/num_radius_divisions:.4f}m")
        
        # Save synthetic PCD
        output_pcd_path = f"output_synthetic_circles_{os.path.basename(file_path).replace('.pcd', '')}.pcd"
        o3d.io.write_point_cloud(output_pcd_path, synthetic_pcd)
        print(f"Synthetic PCD saved to: {output_pcd_path}")

        # Calculate bounding box center for camera lookat point
        aabb = pcd_original.get_axis_aligned_bounding_box()
        center = aabb.get_center()

        print("Visualizing results. Close the window to exit.")
        
        # Add all concentric circles to visualization
        all_geometries = [pcd_original, closest_point_sphere, origin_sphere, line_set, 
                         new_radius_center_sphere, circle_line_set, synthetic_pcd] + concentric_circles
        
        o3d.visualization.draw_geometries(
            all_geometries,
            window_name="Road Analysis with Concentric Synthetic Circles (10 divisions)",
            front=[1, 0, 0],  # Look along positive X-axis (adjust if needed)
            lookat=center,    # Center of the point cloud
            up=[0, 0, 1],     # Z-axis is up
            zoom=0.5          # Adjust zoom level as needed
        )

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    pcd_file = r"ncdb-cls-sample\synced_data\pcd\synthetic_depth_output_test_single_file\new_pcd\0000000931.pcd"
    # These parameters might need further adjustment based on visual inspection.
    find_nearest_road_point_by_height_and_xy_proximity(
        pcd_file, 
        ground_z_min=-0.75, 
        ground_z_max=0.0, 
        min_xy_distance_from_origin=1.0, 
        xy_radius_threshold=10.0,
        y_min=1.7, # New Y-axis filter
        y_max=2.0  # New Y-axis filter
    )
