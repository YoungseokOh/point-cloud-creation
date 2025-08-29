import open3d as o3d
import numpy as np

# Note: The user wants to analyze the structure of a PCD file to find road points.
# This script will load a PCD file and print information about its structure,
# specifically the fields available for each point. This will help identify
# which field contains the semantic label for "road".

def inspect_pcd(file_path):
    """
    Loads a PCD file and prints its structure.
    """
    try:
        pcd = o3d.io.read_point_cloud(file_path)
        
        print(f"Successfully loaded {file_path}")
        print("-" * 30)
        
        # Print the point cloud object itself to see its summary
        print("Point cloud object:")
        print(pcd)
        print("-" * 30)
        
        # Check the available data fields by converting a small part to numpy arrays
        points = np.asarray(pcd.points)
        print(f"Points array shape: {points.shape}")
        
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            print(f"Colors array shape: {colors.shape}")
            print("Note: Colors might represent semantic labels.")
            
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            print(f"Normals array shape: {normals.shape}")

        # In many datasets, semantic labels are stored as colors or in a separate file.
        # If the PCD file is from a common dataset like KITTI or NuScenes,
        # the labels might be encoded in a specific way not immediately obvious.
        # For now, let's assume the structure is self-contained.
        
        # Let's check the first few points' data
        if len(points) > 5:
            print("\nSample of first 5 points (X, Y, Z):")
            print(points[:5])
        
        if pcd.has_colors() and len(colors) > 5:
            print("\nSample of first 5 colors (R, G, B):")
            print(colors[:5])

    except Exception as e:
        print(f"Error reading or inspecting PCD file: {e}")

if __name__ == "__main__":
    # Using the sample file identified earlier
    pcd_file = r"ncdb-cls-sample/synced_data/pcd/0000000931.pcd"
    inspect_pcd(pcd_file)
