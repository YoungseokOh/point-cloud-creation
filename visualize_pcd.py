# Note: The user wants to visually inspect the PCD file to understand its structure,
# especially to see if road points can be identified visually (e.g., by color).

import open3d as o3d
import numpy as np
import os

def visualize_pcd(file_path):
    """
    Loads and visualizes a PCD file using Open3D.
    Also prints the bounding box to check the data's validity.
    """
    if not os.path.exists(file_path):
        print(f"Error: File not found at {file_path}")
        return

    try:
        print(f"Attempting to load {file_path}...")
        pcd = o3d.io.read_point_cloud(file_path)
        
        if not pcd.has_points():
            print("Error: The loaded point cloud has no points.")
            return
            
        print("Successfully loaded point cloud.")
        
        # Get the bounding box of the point cloud
        aabb = pcd.get_axis_aligned_bounding_box()
        min_bound = aabb.get_min_bound()
        max_bound = aabb.get_max_bound()
        
        print("-" * 30)
        print("Point Cloud Axis-Aligned Bounding Box:")
        print(f"Min bound: {min_bound}")
        print(f"Max bound: {max_bound}")
        print("-" * 30)

        # Check if all points are at (0,0,0)
        points = np.asarray(pcd.points)
        if np.all(points == 0):
            print("Warning: All points in the cloud are at (0, 0, 0).")
        
        print("Visualizing point cloud. Close the window to continue...")
        o3d.visualization.draw_geometries([pcd], window_name=f"PCD Viewer: {os.path.basename(file_path)}")
        print("Visualization window closed.")

    except Exception as e:
        print(f"An error occurred during visualization: {e}")

if __name__ == "__main__":
    pcd_file = r"ncdb-cls-sample/synced_data/pcd/0000000931.pcd"
    visualize_pcd(pcd_file)
