#!/usr/bin/env python3
"""
Compare three projection methods:
- Method 1: 1920x1536 Direct (baseline)
- Method 2: 640x512 (scaled with div adjustment)
- Method 3: 640x384 (scaled with div adjustment)

Shows RGB+Depth projection visualization for each method
"""

import cv2
import numpy as np
from pathlib import Path
import sys
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Add project root to path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from integrated_pcd_depth_pipeline_newest import (
    CalibrationDB, 
    LidarCameraProjector,
    VADASFisheyeCameraModel,
    DEFAULT_CALIB,
    DEFAULT_LIDAR_TO_WORLD
)

def load_pcd_simple(pcd_path: Path) -> np.ndarray:
    """Load PCD file and return point cloud as numpy array."""
    with open(pcd_path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            try:
                line_str = line.decode('ascii').strip()
            except:
                break
            header_lines.append(line_str)
            if line_str.startswith('DATA'):
                break
        
        is_binary = False
        num_points = 0
        for line in header_lines:
            if line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                if 'binary' in line.lower():
                    is_binary = True
        
        if is_binary:
            dtype = np.dtype([
                ('x', np.float32), 
                ('y', np.float32), 
                ('z', np.float32), 
                ('intensity', np.float32),
                ('t', np.uint32),
                ('ring', np.uint16)
            ])
            data = np.frombuffer(f.read(), dtype=dtype, count=num_points)
            points = np.column_stack([data['x'], data['y'], data['z']])
            return points
        else:
            points = []
            for line in f:
                try:
                    line_str = line.decode('ascii').strip()
                    if not line_str:
                        continue
                    parts = line_str.split()
                    if len(parts) >= 3:
                        points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                except:
                    continue
            return np.array(points)

def create_rgb_with_depth_scatter(rgb_image: np.ndarray, depth_map: np.ndarray,
                                  point_size: int = 2, max_depth: float = 15.0) -> np.ndarray:
    """Draw depth points on RGB image using OpenCV scatter-style visualization."""
    overlay = rgb_image.copy()
    h, w = depth_map.shape
    
    valid_mask = depth_map > 0
    valid_coords = np.argwhere(valid_mask)
    
    if len(valid_coords) == 0:
        return overlay
    
    depths = depth_map[valid_mask]
    depths_normalized = np.clip(depths / max_depth, 0, 1)
    depths_uint8 = (depths_normalized * 255).astype(np.uint8)
    
    depth_colors_1d = cv2.applyColorMap(depths_uint8.reshape(-1, 1, 1), cv2.COLORMAP_JET)
    depth_colors = depth_colors_1d.reshape(-1, 3)
    
    for (y, x), color in zip(valid_coords, depth_colors):
        color_tuple = (int(color[0]), int(color[1]), int(color[2]))
        cv2.circle(overlay, (int(x), int(y)), point_size, color_tuple, -1)
    
    return overlay

def project_point_with_aspect_ratio(camera_model, Xc, Yc, Zc, scale_x=1.0, scale_y=1.0):
    """
    Project a 3D point to 2D image coordinates with manual aspect ratio scaling.
    
    Based on ref_camera_lidar_projector.py project_point() but with aspect ratio control.
    """
    import math
    import sys
    
    # Normalized coordinates (VADAS expects camera looking along +X axis)
    nx = -Yc
    ny = -Zc
    
    dist = math.hypot(nx, ny)
    if dist < sys.float_info.epsilon:
        dist = sys.float_info.epsilon
    
    cosPhi = nx / dist
    sinPhi = ny / dist
    theta = math.atan2(dist, Xc)
    
    if Xc < 0:
        return 0, 0, False
    
    xd = theta * camera_model.s
    
    if abs(camera_model.div) < 1e-9:
        return 0, 0, False
    
    # Polynomial evaluation
    rd = 0.0
    for c in reversed(camera_model.k):
        rd = rd * xd + c
    rd = rd / camera_model.div
    
    if math.isinf(rd) or math.isnan(rd):
        return 0, 0, False
    
    # Image center offset
    img_w_half = (camera_model.image_size[0] / 2) if camera_model.image_size else 0
    img_h_half = (camera_model.image_size[1] / 2) if camera_model.image_size else 0
    
    # Apply aspect ratio scaling to rd components
    u = rd * cosPhi * scale_x + camera_model.ux + img_w_half
    v = rd * sinPhi * scale_y + camera_model.uy + img_h_half
    
    return int(round(u)), int(round(v)), True

def project_manually(points, sensor_info, calib_db, camera_model, image_size, scale_x=1.0, scale_y=1.0):
    """Manually project points with aspect ratio scaling."""
    depth_map = np.zeros((image_size[1], image_size[0]), dtype=np.float32)
    
    cloud_xyz_hom = np.hstack((points, np.ones((points.shape[0], 1))))
    
    # Apply filtering
    exclude_y_condition = (cloud_xyz_hom[:, 1] <= 0.5) & (cloud_xyz_hom[:, 1] >= -0.7)
    exclude_x_condition = (cloud_xyz_hom[:, 0] >= 0.0)
    points_to_exclude = exclude_y_condition & exclude_x_condition
    cloud_xyz_hom = cloud_xyz_hom[~points_to_exclude]
    
    # Transform to camera coordinates
    lidar_to_camera_transform = sensor_info.extrinsic @ calib_db.lidar_to_world
    points_cam_hom = (lidar_to_camera_transform @ cloud_xyz_hom.T).T
    points_cam = points_cam_hom[:, :3]
    
    # Project points with aspect ratio scaling
    for i in range(points_cam.shape[0]):
        Xc, Yc, Zc = points_cam[i]
        
        if Xc <= 0:
            continue
        
        u, v, valid_projection = project_point_with_aspect_ratio(
            camera_model, Xc, Yc, Zc, scale_x=scale_x, scale_y=scale_y
        )
        
        if valid_projection and 0 <= u < image_size[0] and 0 <= v < image_size[1]:
            if depth_map[v, u] == 0 or depth_map[v, u] > Xc:
                depth_map[v, u] = Xc
    
    return depth_map

def test_div_scaling(
    pcd_path: Path,
    rgb_image_path: Path,
    output_dir: Path,
    image_size_640x512: tuple = (640, 512),
    image_size_640x384: tuple = (640, 384),
    max_depth_display: float = 15.0,
):
    """
    Compare three projection methods showing RGB+Depth overlay
    """
    
    print("=" * 80)
    print("Three-Resolution Projection Comparison")
    print("=" * 80)
    print(f"PCD file:   {pcd_path}")
    print(f"RGB image:  {rgb_image_path}")
    print()
    
    # Load data
    print("Loading point cloud...")
    points = load_pcd_simple(pcd_path)
    print(f"  Loaded {len(points):,} points")
    
    print("Loading RGB image...")
    rgb_image_original = cv2.imread(str(rgb_image_path))
    if rgb_image_original is None:
        print(f"Error: Could not load image from {rgb_image_path}")
        return
    
    output_dir.mkdir(parents=True, exist_ok=True)
    sample_name = pcd_path.stem
    
    # ===== METHOD 1: 1920x1536 (Baseline) =====
    print("\n" + "=" * 80)
    print("METHOD 1: 1920x1536 Direct Projection (Baseline)")
    print("=" * 80)
    
    calib_db_1 = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD)
    sensor_info_1 = calib_db_1.get("a6")
    camera_model_1 = sensor_info_1.model
    
    # No scaling for original resolution
    camera_model_1.image_size = (1920, 1536)
    
    # Use manual projection with scale_x=1.0, scale_y=1.0 (no scaling)
    depth_map_1 = project_manually(
        points, sensor_info_1, calib_db_1, camera_model_1, (1920, 1536),
        scale_x=1.0, scale_y=1.0
    )
    
    if depth_map_1 is None:
        print("Error: Depth map projection failed for Method 1")
        return
    
    valid_pixels_1 = np.sum(depth_map_1 > 0)
    coverage_1 = (valid_pixels_1 / (1920 * 1536)) * 100
    
    print(f"  Valid pixels: {valid_pixels_1:,}")
    print(f"  Coverage: {coverage_1:.2f}%")
    
    if valid_pixels_1 > 0:
        valid_depths_1 = depth_map_1[depth_map_1 > 0]
        print(f"  Depth range: {valid_depths_1.min():.2f}m - {valid_depths_1.max():.2f}m")
        print(f"  Mean depth: {valid_depths_1.mean():.2f}m")
    
    # RGB for 1920x1536
    rgb_image_1920 = rgb_image_original.copy()
    
    # ===== METHOD 2: 640x512 (with aspect ratio scaling) =====
    print("\n" + "=" * 80)
    print("METHOD 2: 640x512 with aspect ratio scaling")
    print("=" * 80)
    
    calib_db_2 = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD)
    sensor_info_2 = calib_db_2.get("a6")
    camera_model_2 = sensor_info_2.model
    
    scale_x_512 = image_size_640x512[0] / 1920.0
    scale_y_512 = image_size_640x512[1] / 1536.0
    
    print(f"  Scale factors: scale_x={scale_x_512:.6f}, scale_y={scale_y_512:.6f}")
    
    # Apply intrinsic scaling (ux, uy만 스케일, div는 원본 유지)
    camera_model_2.ux = camera_model_2.original_intrinsic[9] * scale_x_512
    camera_model_2.uy = camera_model_2.original_intrinsic[10] * scale_y_512
    camera_model_2.div = camera_model_2.original_intrinsic[8]  # 원본 유지!
    camera_model_2.image_size = image_size_640x512
    
    print(f"  Original div: {camera_model_2.original_intrinsic[8]}")
    print(f"  Used div: {camera_model_2.div} (unchanged)")
    
    # Use manual projection with aspect ratio scaling
    depth_map_2 = project_manually(
        points, sensor_info_2, calib_db_2, camera_model_2, image_size_640x512,
        scale_x=scale_x_512, scale_y=scale_y_512
    )
    
    valid_pixels_2 = np.sum(depth_map_2 > 0)
    coverage_2 = (valid_pixels_2 / (image_size_640x512[0] * image_size_640x512[1])) * 100
    
    print(f"  Valid pixels: {valid_pixels_2:,}")
    print(f"  Coverage: {coverage_2:.2f}%")
    
    if valid_pixels_2 > 0:
        valid_depths_2 = depth_map_2[depth_map_2 > 0]
        print(f"  Depth range: {valid_depths_2.min():.2f}m - {valid_depths_2.max():.2f}m")
        print(f"  Mean depth: {valid_depths_2.mean():.2f}m")
    
    # RGB for 640x512
    rgb_image_512 = cv2.resize(rgb_image_original, image_size_640x512, interpolation=cv2.INTER_AREA)
    
    # ===== METHOD 3: 640x384 (with aspect ratio scaling) =====
    print("\n" + "=" * 80)
    print("METHOD 3: 640x384 with aspect ratio scaling")
    print("=" * 80)
    
    calib_db_3 = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD)
    sensor_info_3 = calib_db_3.get("a6")
    camera_model_3 = sensor_info_3.model
    
    scale_x_384 = image_size_640x384[0] / 1920.0
    scale_y_384 = image_size_640x384[1] / 1536.0
    
    print(f"  Scale factors: scale_x={scale_x_384:.6f}, scale_y={scale_y_384:.6f}")
    
    # Apply intrinsic scaling (ux, uy만 스케일, div는 원본 유지)
    camera_model_3.ux = camera_model_3.original_intrinsic[9] * scale_x_384
    camera_model_3.uy = camera_model_3.original_intrinsic[10] * scale_y_384
    camera_model_3.div = camera_model_3.original_intrinsic[8]  # 원본 유지!
    camera_model_3.image_size = image_size_640x384
    
    print(f"  Original div: {camera_model_3.original_intrinsic[8]}")
    print(f"  Used div: {camera_model_3.div} (unchanged)")
    
    # Use manual projection with aspect ratio scaling
    depth_map_3 = project_manually(
        points, sensor_info_3, calib_db_3, camera_model_3, image_size_640x384,
        scale_x=scale_x_384, scale_y=scale_y_384
    )
    
    valid_pixels_3 = np.sum(depth_map_3 > 0)
    coverage_3 = (valid_pixels_3 / (image_size_640x384[0] * image_size_640x384[1])) * 100
    
    print(f"  Valid pixels: {valid_pixels_3:,}")
    print(f"  Coverage: {coverage_3:.2f}%")
    
    if valid_pixels_3 > 0:
        valid_depths_3 = depth_map_3[depth_map_3 > 0]
        print(f"  Depth range: {valid_depths_3.min():.2f}m - {valid_depths_3.max():.2f}m")
        print(f"  Mean depth: {valid_depths_3.mean():.2f}m")
    
    # ===== CREATE MATPLOTLIB VISUALIZATION =====
    print("\nGenerating matplotlib visualization...")
    
    fig = plt.figure(figsize=(18, 6))
    gs = GridSpec(1, 3, figure=fig, wspace=0.15)
    
    # Title
    fig.suptitle(
        f'RGB+Depth Projection Comparison: {sample_name}',
        fontsize=14, fontweight='bold'
    )
    
    # ===== ROW 1: RGB + Depth Scatter for each resolution =====
    
    # Method 1: 1920x1536 RGB + depth scatter
    ax1 = fig.add_subplot(gs[0, 0])
    rgb_with_depth_1 = create_rgb_with_depth_scatter(
        rgb_image_1920.copy(), depth_map_1, point_size=4, max_depth=max_depth_display
    )
    # Resize for display
    rgb_with_depth_1_display = cv2.resize(rgb_with_depth_1, (512, 384), interpolation=cv2.INTER_AREA)
    ax1.imshow(cv2.cvtColor(rgb_with_depth_1_display, cv2.COLOR_BGR2RGB))
    ax1.set_title(f'1920×1536 (Baseline)\n{valid_pixels_1:,} px | {coverage_1:.1f}% | {valid_depths_1.mean():.2f}m', 
                  fontsize=11, fontweight='bold')
    ax1.axis('off')
    
    # Method 2: 640x512 RGB + depth scatter
    ax2 = fig.add_subplot(gs[0, 1])
    rgb_with_depth_2 = create_rgb_with_depth_scatter(
        rgb_image_512.copy(), depth_map_2, point_size=2, max_depth=max_depth_display
    )
    ax2.imshow(cv2.cvtColor(rgb_with_depth_2, cv2.COLOR_BGR2RGB))
    ax2.set_title(f'640×512 (aspect ratio)\n{valid_pixels_2:,} px | {coverage_2:.1f}% | {valid_depths_2.mean():.2f}m', 
                  fontsize=11, fontweight='bold', color='green')
    ax2.axis('off')
    
    # Method 3: 640x384 RGB + depth scatter
    ax3 = fig.add_subplot(gs[0, 2])
    rgb_image_384 = cv2.resize(rgb_image_original, image_size_640x384, interpolation=cv2.INTER_AREA)
    rgb_with_depth_3 = create_rgb_with_depth_scatter(
        rgb_image_384.copy(), depth_map_3, point_size=2, max_depth=max_depth_display
    )
    ax3.imshow(cv2.cvtColor(rgb_with_depth_3, cv2.COLOR_BGR2RGB))
    ax3.set_title(f'640×384 (aspect ratio)\n{valid_pixels_3:,} px | {coverage_3:.1f}% | {valid_depths_3.mean():.2f}m', 
                  fontsize=11, fontweight='bold')
    ax3.axis('off')
    
    # Save figure
    output_path = output_dir / f"{sample_name}_resolution_comparison.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\n✅ Saved comparison: {output_path.name}")
    
    print("\n" + "=" * 80)
    print("Comparison complete!")
    print(f"Results saved to: {output_dir}")
    print("=" * 80)

def main():
    """Main entry point."""
    
    PROJECT_ROOT = Path(__file__).parent
    sample_data_dir = PROJECT_ROOT / "ncdb-cls-sample" / "synced_data"
    pcd_dir = sample_data_dir / "pcd"
    rgb_dir = sample_data_dir / "image_a6"
    
    if len(sys.argv) > 1:
        sample_id = sys.argv[1]
        print(f"Using sample ID from argument: {sample_id}")
    else:
        sample_id = "0000000931"
    
    pcd_path = pcd_dir / f"{sample_id}.pcd"
    rgb_path = rgb_dir / f"{sample_id}.jpg"
    
    if not rgb_path.exists():
        rgb_path = rgb_dir / f"{sample_id}.png"
    
    output_dir = PROJECT_ROOT / "output" / "test_640x384_projection"
    
    if not pcd_path.exists():
        print(f"Error: PCD file not found: {pcd_path}")
        return
    
    if not rgb_path.exists():
        print(f"Error: RGB image not found: {rgb_path}")
        return
    
    test_div_scaling(
        pcd_path=pcd_path,
        rgb_image_path=rgb_path,
        output_dir=output_dir,
        image_size_640x512=(640, 512),
        image_size_640x384=(640, 384),
        max_depth_display=15.0,
    )

if __name__ == "__main__":
    main()
