#!/usr/bin/env python3
"""
Verify 640x384 depth map alignment with RGB image by direct visualization
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

from integrated_pcd_depth_pipeline_v3 import (
    CalibrationDB, 
    LidarCameraProjector,
    DEFAULT_CALIB,
    DEFAULT_LIDAR_TO_WORLD
)

def load_pcd_simple(pcd_path: Path) -> np.ndarray:
    """Load PCD file and return point cloud as numpy array."""
    # Read header to determine format
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
        
        # Parse header
        is_binary = False
        num_points = 0
        for line in header_lines:
            if line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                if 'binary' in line.lower():
                    is_binary = True
        
        if is_binary:
            # Read binary data
            # VADAS PCD format: x y z intensity t ring
            # SIZE: 4 4 4 4 4 2 (bytes)
            # TYPE: F F F F U U (float32, float32, float32, float32, uint32, uint16)
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
            # ASCII format
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

def create_depth_overlay(rgb_image: np.ndarray, depth_map: np.ndarray, 
                        alpha: float = 0.6, max_depth: float = 15.0) -> np.ndarray:
    """Create overlay of depth map on RGB image."""
    # Normalize depth for visualization
    depth_vis = depth_map.copy()
    depth_vis[depth_vis > max_depth] = max_depth
    depth_vis = (depth_vis / max_depth * 255).astype(np.uint8)
    
    # Apply JET colormap
    depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    
    # Set background (zero depth) to black
    mask = depth_map == 0
    depth_colored[mask] = [0, 0, 0]
    
    # Create overlay
    overlay = cv2.addWeighted(rgb_image, 1-alpha, depth_colored, alpha, 0)
    overlay[mask] = rgb_image[mask]  # Keep original RGB where no depth
    
    return overlay

def create_depth_points_overlay(rgb_image: np.ndarray, depth_map: np.ndarray,
                                point_size: int = 2, max_depth: float = 15.0) -> np.ndarray:
    """Draw depth points on RGB image with depth-based coloring."""
    overlay = rgb_image.copy()
    
    # Get valid depth pixels
    valid_mask = depth_map > 0
    valid_coords = np.argwhere(valid_mask)  # Returns [row, col] = [y, x]
    
    if len(valid_coords) == 0:
        return overlay
    
    # Get depth values
    depths = depth_map[valid_mask]
    
    # Normalize depths for coloring
    depths_normalized = np.clip(depths / max_depth, 0, 1)
    
    # Create colormap (JET)
    colors = plt.cm.jet(depths_normalized)[:, :3] * 255  # RGB, 0-255
    
    # Draw points
    for (y, x), color in zip(valid_coords, colors):
        cv2.circle(overlay, (int(x), int(y)), point_size, color.tolist(), -1)
    
    return overlay

def create_comparison_visualization(rgb_image: np.ndarray, depth_map: np.ndarray,
                                   sample_name: str, output_path: Path,
                                   max_depth: float = 15.0):
    """Create comprehensive comparison visualization."""
    
    # Calculate statistics
    valid_depths = depth_map[depth_map > 0]
    if len(valid_depths) == 0:
        print(f"Warning: No valid depth values for {sample_name}")
        return
    
    min_depth = valid_depths.min()
    max_depth_actual = valid_depths.max()
    mean_depth = valid_depths.mean()
    coverage = (len(valid_depths) / depth_map.size) * 100
    
    # Create visualizations
    depth_overlay = create_depth_overlay(rgb_image, depth_map, alpha=0.5, max_depth=max_depth)
    depth_points = create_depth_points_overlay(rgb_image, depth_map, point_size=2, max_depth=max_depth)
    
    # Depth map colored
    depth_vis = depth_map.copy()
    depth_vis[depth_vis > max_depth] = max_depth
    depth_normalized = (depth_vis / max_depth * 255).astype(np.uint8)
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
    depth_colored[depth_map == 0] = [255, 255, 255]  # White background
    
    # Create figure with matplotlib
    fig = plt.figure(figsize=(20, 12))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.2)
    
    # Title
    fig.suptitle(f'Depth-RGB Alignment Verification: {sample_name}\n' +
                 f'Resolution: {rgb_image.shape[1]}x{rgb_image.shape[0]} | ' +
                 f'Coverage: {coverage:.1f}% | ' +
                 f'Depth Range: {min_depth:.2f}m - {max_depth_actual:.2f}m (mean: {mean_depth:.2f}m)',
                 fontsize=16, fontweight='bold')
    
    # Row 1: Original RGB, Depth Map, Overlay (50%)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.imshow(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
    ax1.set_title('Original RGB Image (640x384)', fontsize=12, fontweight='bold')
    ax1.axis('off')
    
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.imshow(cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB))
    ax2.set_title(f'Depth Map (JET colormap, max={max_depth}m)', fontsize=12, fontweight='bold')
    ax2.axis('off')
    
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.imshow(cv2.cvtColor(depth_overlay, cv2.COLOR_BGR2RGB))
    ax3.set_title('Depth Overlay (50% transparency)', fontsize=12, fontweight='bold')
    ax3.axis('off')
    
    # Row 2: Point overlay, Depth histogram, Coverage map
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.imshow(cv2.cvtColor(depth_points, cv2.COLOR_BGR2RGB))
    ax4.set_title('Depth Points on RGB (colored by depth)', fontsize=12, fontweight='bold')
    ax4.axis('off')
    
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.hist(valid_depths, bins=50, color='steelblue', edgecolor='black', alpha=0.7)
    ax5.set_xlabel('Depth (meters)', fontsize=10)
    ax5.set_ylabel('Pixel Count', fontsize=10)
    ax5.set_title('Depth Distribution', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    
    ax6 = fig.add_subplot(gs[1, 2])
    coverage_map = (depth_map > 0).astype(np.uint8) * 255
    ax6.imshow(coverage_map, cmap='gray')
    ax6.set_title(f'Depth Coverage Map ({coverage:.1f}%)', fontsize=12, fontweight='bold')
    ax6.axis('off')
    
    # Row 3: Zoomed regions (center, left edge, right edge)
    h, w = rgb_image.shape[:2]
    
    # Center zoom
    center_x, center_y = w // 2, h // 2
    zoom_size = 100
    x1, x2 = max(0, center_x - zoom_size), min(w, center_x + zoom_size)
    y1, y2 = max(0, center_y - zoom_size), min(h, center_y + zoom_size)
    
    ax7 = fig.add_subplot(gs[2, 0])
    center_zoom = depth_points[y1:y2, x1:x2]
    ax7.imshow(cv2.cvtColor(center_zoom, cv2.COLOR_BGR2RGB))
    ax7.set_title(f'Center Zoom ({x1}:{x2}, {y1}:{y2})', fontsize=12, fontweight='bold')
    ax7.axis('off')
    
    # Left edge zoom
    left_x = w // 4
    x1, x2 = max(0, left_x - zoom_size), min(w, left_x + zoom_size)
    
    ax8 = fig.add_subplot(gs[2, 1])
    left_zoom = depth_points[y1:y2, x1:x2]
    ax8.imshow(cv2.cvtColor(left_zoom, cv2.COLOR_BGR2RGB))
    ax8.set_title(f'Left Region Zoom ({x1}:{x2}, {y1}:{y2})', fontsize=12, fontweight='bold')
    ax8.axis('off')
    
    # Right edge zoom
    right_x = 3 * w // 4
    x1, x2 = max(0, right_x - zoom_size), min(w, right_x + zoom_size)
    
    ax9 = fig.add_subplot(gs[2, 2])
    right_zoom = depth_points[y1:y2, x1:x2]
    ax9.imshow(cv2.cvtColor(right_zoom, cv2.COLOR_BGR2RGB))
    ax9.set_title(f'Right Region Zoom ({x1}:{x2}, {y1}:{y2})', fontsize=12, fontweight='bold')
    ax9.axis('off')
    
    # Save figure
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"✅ Saved visualization: {output_path}")
    print(f"   Statistics: {len(valid_depths):,} valid pixels ({coverage:.1f}% coverage)")
    print(f"   Depth range: {min_depth:.2f}m - {max_depth_actual:.2f}m (mean: {mean_depth:.2f}m)")

def verify_depth_rgb_alignment(
    pcd_path: Path,
    rgb_image_path: Path,
    output_dir: Path,
    image_size: tuple = (640, 384),
    max_depth_display: float = 15.0
):
    """Main verification function."""
    
    print("=" * 80)
    print("Depth-RGB Alignment Verification")
    print("=" * 80)
    print(f"PCD file:   {pcd_path}")
    print(f"RGB image:  {rgb_image_path}")
    print(f"Resolution: {image_size[0]}x{image_size[1]}")
    print()
    
    # Load data
    print("Loading point cloud...")
    points = load_pcd_simple(pcd_path)
    print(f"  Loaded {len(points):,} points")
    
    print("Loading RGB image...")
    rgb_image = cv2.imread(str(rgb_image_path))
    if rgb_image is None:
        print(f"Error: Could not load image from {rgb_image_path}")
        return
    
    # Resize if needed
    if (rgb_image.shape[1], rgb_image.shape[0]) != image_size:
        print(f"  Resizing from {rgb_image.shape[1]}x{rgb_image.shape[0]} to {image_size[0]}x{image_size[1]}")
        rgb_image = cv2.resize(rgb_image, image_size, interpolation=cv2.INTER_AREA)
    
    print(f"  Image shape: {rgb_image.shape}")
    
    # Create depth map
    calib_db = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD)
    projector = LidarCameraProjector(calib_db)
    
    # === TEST: Compare direct projection vs resize method ===
    print("\n" + "=" * 80)
    print("COMPARISON TEST: Direct 640x384 vs Resized from 1920x1536")
    print("=" * 80)
    
    # Method 2: Project to 1920x1536 then resize (do this FIRST to avoid state issues)
    print("\n[Method 2] Project to 1920x1536 then resize...")
    projector_1920 = LidarCameraProjector(calib_db)  # Fresh projector
    depth_map_1920 = projector_1920.project_cloud_to_depth_map("a6", points, (1920, 1536))
    valid_1920 = np.sum(depth_map_1920 > 0)
    print(f"  Valid pixels at 1920x1536: {valid_1920:,}")
    
    # Method 1: Direct projection to 640x384 (current method)
    print("\n[Method 1] Direct projection to 640x384...")
    projector_640 = LidarCameraProjector(calib_db)  # Fresh projector
    depth_map_direct = projector_640.project_cloud_to_depth_map("a6", points, (640, 384))
    valid_direct = np.sum(depth_map_direct > 0)
    print(f"  Valid pixels: {valid_direct:,}")
    
    # Resize depth map (nearest neighbor to preserve exact depth values)
    depth_map_resized = cv2.resize(depth_map_1920, (640, 384), interpolation=cv2.INTER_NEAREST)
    valid_resized = np.sum(depth_map_resized > 0)
    print(f"  Valid pixels after resize: {valid_resized:,}")
    
    # Compare the two depth maps
    print("\n[Comparison]")
    
    # Pixel-wise difference
    diff_mask = (depth_map_direct > 0) | (depth_map_resized > 0)
    diff_map = np.abs(depth_map_direct - depth_map_resized)
    diff_map[~diff_mask] = 0
    
    valid_both = (depth_map_direct > 0) & (depth_map_resized > 0)
    if np.any(valid_both):
        diff_values = diff_map[valid_both]
        print(f"  Pixels valid in both: {np.sum(valid_both):,}")
        print(f"  Mean difference: {diff_values.mean():.4f} meters")
        print(f"  Max difference: {diff_values.max():.4f} meters")
        print(f"  Std difference: {diff_values.std():.4f} meters")
        
        # Positional shift analysis
        only_direct = (depth_map_direct > 0) & (depth_map_resized == 0)
        only_resized = (depth_map_direct == 0) & (depth_map_resized > 0)
        print(f"  Pixels only in direct: {np.sum(only_direct):,}")
        print(f"  Pixels only in resized: {np.sum(only_resized):,}")
        
        # Calculate spatial shift (check if points moved)
        shift_metric = np.sum(only_direct) + np.sum(only_resized)
        total_pixels = np.sum(diff_mask)
        shift_percent = (shift_metric / total_pixels * 100) if total_pixels > 0 else 0
        print(f"  Position shift metric: {shift_percent:.2f}% pixels differ")
    
    # Save comparison visualization
    print("\n[Saving comparison images...]")
    sample_name = pcd_path.stem
    
    # Load original 1920x1536 RGB for comparison
    rgb_1920 = cv2.imread(str(rgb_image_path))
    
    # Create visualizations for 1920x1536
    # Point size = 3 for 1920x1536
    rgb_1920_with_points = create_depth_points_overlay(
        rgb_1920.copy(), depth_map_1920, point_size=3, max_depth=max_depth_display
    )
    
    # Resize 1920x1536 visualization to 640x384 for comparison display
    rgb_1920_resized_vis = cv2.resize(rgb_1920_with_points, (640, 384), interpolation=cv2.INTER_AREA)
    
    # Create visualizations for 640x384 methods
    # Point size = 1 for 640x384 (scale: 640/1920 = 1/3, so 3*1/3 = 1)
    rgb_direct_vis = create_depth_points_overlay(
        rgb_image.copy(), depth_map_direct, point_size=1, max_depth=max_depth_display
    )
    rgb_resized_vis = create_depth_points_overlay(
        rgb_image.copy(), depth_map_resized, point_size=1, max_depth=max_depth_display
    )
    
    # Create comprehensive comparison figure
    fig = plt.figure(figsize=(20, 15))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.2)
    
    # Title with statistics
    fig.suptitle(
        f'Resolution Comparison: {sample_name}\n'
        f'1920x1536 projection: {valid_1920:,} pixels | '
        f'640x384 direct: {valid_direct:,} pixels | '
        f'640x384 resized: {valid_resized:,} pixels',
        fontsize=16, fontweight='bold'
    )
    
    # Row 1: Three methods side by side
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.imshow(cv2.cvtColor(rgb_1920_resized_vis, cv2.COLOR_BGR2RGB))
    ax1.set_title(f'1920x1536 Projection\n({valid_1920:,} pixels)\n[Resized to 640x384 for display]', 
                  fontsize=12, fontweight='bold')
    ax1.axis('off')
    
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.imshow(cv2.cvtColor(rgb_direct_vis, cv2.COLOR_BGR2RGB))
    ax2.set_title(f'640x384 Direct Projection\n({valid_direct:,} pixels)\n✅ RECOMMENDED', 
                  fontsize=12, fontweight='bold', color='green')
    ax2.axis('off')
    
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.imshow(cv2.cvtColor(rgb_resized_vis, cv2.COLOR_BGR2RGB))
    ax3.set_title(f'640x384 from Resized Depth\n({valid_resized:,} pixels)\n❌ NOT RECOMMENDED', 
                  fontsize=12, fontweight='bold', color='red')
    ax3.axis('off')
    
    # Row 2: Depth maps colored
    depth_1920_vis = depth_map_1920.copy()
    depth_1920_vis[depth_1920_vis > max_depth_display] = max_depth_display
    depth_1920_normalized = (depth_1920_vis / max_depth_display * 255).astype(np.uint8)
    depth_1920_colored = cv2.applyColorMap(depth_1920_normalized, cv2.COLORMAP_JET)
    depth_1920_colored[depth_map_1920 == 0] = [255, 255, 255]
    depth_1920_colored_resized = cv2.resize(depth_1920_colored, (640, 384), interpolation=cv2.INTER_AREA)
    
    depth_direct_vis = depth_map_direct.copy()
    depth_direct_vis[depth_direct_vis > max_depth_display] = max_depth_display
    depth_direct_normalized = (depth_direct_vis / max_depth_display * 255).astype(np.uint8)
    depth_direct_colored = cv2.applyColorMap(depth_direct_normalized, cv2.COLORMAP_JET)
    depth_direct_colored[depth_map_direct == 0] = [255, 255, 255]
    
    depth_resized_vis = depth_map_resized.copy()
    depth_resized_vis[depth_resized_vis > max_depth_display] = max_depth_display
    depth_resized_normalized = (depth_resized_vis / max_depth_display * 255).astype(np.uint8)
    depth_resized_colored = cv2.applyColorMap(depth_resized_normalized, cv2.COLORMAP_JET)
    depth_resized_colored[depth_map_resized == 0] = [255, 255, 255]
    
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.imshow(cv2.cvtColor(depth_1920_colored_resized, cv2.COLOR_BGR2RGB))
    ax4.set_title('1920x1536 Depth Map\n[Resized for display]', fontsize=11)
    ax4.axis('off')
    
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.imshow(cv2.cvtColor(depth_direct_colored, cv2.COLOR_BGR2RGB))
    ax5.set_title('640x384 Direct Depth Map', fontsize=11)
    ax5.axis('off')
    
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.imshow(cv2.cvtColor(depth_resized_colored, cv2.COLOR_BGR2RGB))
    ax6.set_title('640x384 Resized Depth Map', fontsize=11)
    ax6.axis('off')
    
    # Row 3: Difference analysis
    ax7 = fig.add_subplot(gs[2, 0])
    diff_1920_direct = np.abs(
        cv2.resize(depth_map_1920, (640, 384), interpolation=cv2.INTER_NEAREST) - depth_map_direct
    )
    diff_mask_1920 = (cv2.resize(depth_map_1920, (640, 384), interpolation=cv2.INTER_NEAREST) > 0) | (depth_map_direct > 0)
    diff_1920_direct[~diff_mask_1920] = 0
    diff_1920_vis = (diff_1920_direct / max(diff_1920_direct.max(), 1e-6) * 255).astype(np.uint8)
    diff_1920_colored = cv2.applyColorMap(diff_1920_vis, cv2.COLORMAP_HOT)
    diff_1920_colored[~diff_mask_1920] = [255, 255, 255]
    ax7.imshow(cv2.cvtColor(diff_1920_colored, cv2.COLOR_BGR2RGB))
    if diff_1920_direct[diff_mask_1920].size > 0:
        mean_diff = diff_1920_direct[diff_mask_1920].mean()
        ax7.set_title(f'Diff: 1920 Resized vs 640 Direct\nMean: {mean_diff:.2f}m', fontsize=11)
    else:
        ax7.set_title('Diff: 1920 Resized vs 640 Direct', fontsize=11)
    ax7.axis('off')
    
    ax8 = fig.add_subplot(gs[2, 1])
    if np.any(valid_both):
        ax8.hist(diff_values, bins=30, color='steelblue', edgecolor='black', alpha=0.7)
        ax8.set_xlabel('Depth Difference (m)', fontsize=10)
        ax8.set_ylabel('Pixel Count', fontsize=10)
        ax8.set_title(f'Direct vs Resized Difference\nMean: {diff_values.mean():.2f}m', fontsize=11)
        ax8.grid(True, alpha=0.3)
    else:
        ax8.text(0.5, 0.5, 'No overlapping pixels', ha='center', va='center', fontsize=12)
        ax8.set_title('Direct vs Resized Difference', fontsize=11)
    ax8.set_xlim(0, max_depth_display)
    
    ax9 = fig.add_subplot(gs[2, 2])
    diff_vis_final = (diff_map / max(diff_map.max(), 1e-6) * 255).astype(np.uint8)
    diff_colored_final = cv2.applyColorMap(diff_vis_final, cv2.COLORMAP_HOT)
    diff_colored_final[~diff_mask] = [255, 255, 255]
    ax9.imshow(cv2.cvtColor(diff_colored_final, cv2.COLOR_BGR2RGB))
    ax9.set_title(f'Difference Map\n{shift_percent:.1f}% pixels differ', fontsize=11)
    ax9.axis('off')
    
    # Save comprehensive comparison
    comparison_path = output_dir / f"{sample_name}_full_comparison.png"
    plt.savefig(comparison_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  ✅ Saved comprehensive comparison: {sample_name}_full_comparison.png")
    
    # Save full resolution 1920x1536 result
    rgb_1920_full_path = output_dir / f"{sample_name}_1920x1536_points.png"
    cv2.imwrite(str(rgb_1920_full_path), rgb_1920_with_points)
    print(f"  ✅ Saved 1920x1536 full resolution: {sample_name}_1920x1536_points.png")
    
    depth_1920_full_path = output_dir / f"{sample_name}_1920x1536_depth.png"
    cv2.imwrite(str(depth_1920_full_path), depth_1920_colored)
    print(f"  ✅ Saved 1920x1536 depth map: {sample_name}_1920x1536_depth.png")
    
    print("=" * 80)
    
    # Use direct projection for main visualization
    depth_map = depth_map_direct
    valid_pixels = valid_direct
    print(f"\nUsing Method 1 (direct projection) for main visualization")
    print(f"  Valid pixels: {valid_pixels:,}")
    
    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate visualizations
    sample_name = pcd_path.stem
    output_path = output_dir / f"{sample_name}_verification.png"
    
    print("\nGenerating visualization...")
    create_comparison_visualization(
        rgb_image, 
        depth_map, 
        sample_name, 
        output_path,
        max_depth=max_depth_display
    )
    
    # Save individual outputs
    print("\nSaving individual outputs...")
    
    # Depth overlay
    overlay = create_depth_overlay(rgb_image, depth_map, alpha=0.5, max_depth=max_depth_display)
    cv2.imwrite(str(output_dir / f"{sample_name}_overlay.png"), overlay)
    print(f"  ✅ Saved: {sample_name}_overlay.png")
    
    # Depth points
    points_overlay = create_depth_points_overlay(rgb_image, depth_map, point_size=2, max_depth=max_depth_display)
    cv2.imwrite(str(output_dir / f"{sample_name}_points.png"), points_overlay)
    print(f"  ✅ Saved: {sample_name}_points.png")
    
    # Depth map colored
    depth_vis = depth_map.copy()
    depth_vis[depth_vis > max_depth_display] = max_depth_display
    depth_normalized = (depth_vis / max_depth_display * 255).astype(np.uint8)
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
    depth_colored[depth_map == 0] = [255, 255, 255]
    cv2.imwrite(str(output_dir / f"{sample_name}_depth.png"), depth_colored)
    print(f"  ✅ Saved: {sample_name}_depth.png")
    
    print("\n" + "=" * 80)
    print("Verification complete! Check the output images for alignment quality.")
    print("=" * 80)

def main():
    """Main entry point."""
    import sys
    
    # Sample data paths
    PROJECT_ROOT = Path(__file__).parent
    
    # Example 1: Using ncdb-cls-sample data
    sample_data_dir = PROJECT_ROOT / "ncdb-cls-sample" / "synced_data"
    pcd_dir = sample_data_dir / "pcd"
    rgb_dir = sample_data_dir / "image_a6"
    
    # Allow sample ID from command line argument
    if len(sys.argv) > 1:
        sample_id = sys.argv[1]
        print(f"Using sample ID from argument: {sample_id}")
    else:
        sample_id = "0000000931"  # Default
    
    pcd_path = pcd_dir / f"{sample_id}.pcd"
    rgb_path = rgb_dir / f"{sample_id}.jpg"
    
    # Alternative: Check for .png extension
    if not rgb_path.exists():
        rgb_path = rgb_dir / f"{sample_id}.png"
    
    output_dir = PROJECT_ROOT / "output" / "depth_rgb_verification"
    
    # Verify files exist
    if not pcd_path.exists():
        print(f"Error: PCD file not found: {pcd_path}")
        print("\nAvailable PCD files:")
        if pcd_dir.exists():
            for f in sorted(pcd_dir.glob("*.pcd"))[:5]:
                print(f"  - {f.name}")
        return
    
    if not rgb_path.exists():
        print(f"Error: RGB image not found: {rgb_path}")
        print("\nAvailable image files:")
        if rgb_dir.exists():
            for f in sorted(rgb_dir.glob("*.jpg"))[:5]:
                print(f"  - {f.name}")
            for f in sorted(rgb_dir.glob("*.png"))[:5]:
                print(f"  - {f.name}")
        return
    
    # Run verification
    verify_depth_rgb_alignment(
        pcd_path=pcd_path,
        rgb_image_path=rgb_path,
        output_dir=output_dir,
        image_size=(640, 384),
        max_depth_display=15.0
    )

if __name__ == "__main__":
    main()
