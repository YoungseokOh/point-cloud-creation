#!/usr/bin/env python3
"""
Indoor Dataset v4 Calibration Verification Script

v4 캘리브레이션 검증:
- synchronized_data_indoor_optimized 데이터셋용
- extrinsic: [0.00660233, 0.0122275, -0.230106, -0.0429939, 0.0809541, 0.01427919]
- X 양수가 전방

사용법:
    python indoor_verification.py
    python indoor_verification.py --frame 0000020100
    python indoor_verification.py --random 5
"""

import cv2
import numpy as np
from pathlib import Path
import sys
import argparse
import math
import random

# =============================================================================
# v4 Calibration (X 양수가 전방) - synchronized_data_indoor_optimized용
# =============================================================================
DEFAULT_CALIB_v4 = {
    "a6": {
        "model": "vadas",
        "intrinsic": [
            -0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,  # u2d (k[0:7])
            1.05007, 0.0021, -0.00152588, 0.000793457,  # s, div, ux, uy
            0, 0.9965, -0.0067, -0.0956, 0.1006, -0.054, 0.0106  # d2u
        ],
        "extrinsic": [0.00660233, 0.0122275, -0.230106, -0.0429939, 0.0809541, 0.01427919],
        "image_size": (1920, 1536)
    }
}

# v4용 LiDAR → World 변환 행렬
# LiDAR = World 가정 (Identity)
DEFAULT_LIDAR_TO_WORLD_v4 = np.array([
       [0.999823, -0.0183399, -0.00420554, -0.0510422],
       [0.0182813, 0.999741, -0.0135742, -0.0243783],
       [0.0044534, 0.013495, 0.999899, 0.341502],
       [0, 0, 0, 1]
])

# v4 LiDAR → Camera 변환 행렬
# v4 extrinsic만 사용: LiDAR → Camera = World → Camera (LiDAR=World 가정)
# extrinsic: [0.00660233, 0.0122275, -0.230106, -0.0429939, 0.0809541, 0.01427919]
DEFAULT_LIDAR_TO_CAM_v4 = np.array([
       [0.996513, -0.0331847, 0.076549, -0.0163759],
       [0.0309737, 0.999073, 0.0298925, 0.00209386],
       [-0.07747, -0.0274172, 0.996618, 0.115137],
       [0, 0, 0, 1]
])


# 직접 LiDAR to Camera 매트릭스 사용 여부
# False로 두면 extrinsic @ DEFAULT_LIDAR_TO_WORLD_v4 로 계산
USE_DIRECT_LIDAR_TO_CAM = False


def parse_pcd_file(pcd_path: Path) -> np.ndarray:
    """Parse PCD file (ASCII or BINARY) and return points as Nx3 array."""
    try:
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
            fields = []
            sizes = []
            types = []
            
            for line in header_lines:
                if line.startswith('POINTS'):
                    num_points = int(line.split()[1])
                elif line.startswith('FIELDS'):
                    fields = line.split()[1:]
                elif line.startswith('SIZE'):
                    sizes = [int(x) for x in line.split()[1:]]
                elif line.startswith('TYPE'):
                    types = line.split()[1:]
                elif line.startswith('DATA'):
                    if 'binary' in line.lower():
                        is_binary = True
            
            if is_binary:
                # Build dtype based on header
                dtype_list = []
                for i, (field, size, typ) in enumerate(zip(fields, sizes, types)):
                    if typ == 'F':
                        if size == 4:
                            dtype_list.append((field, np.float32))
                        elif size == 8:
                            dtype_list.append((field, np.float64))
                    elif typ == 'U':
                        if size == 4:
                            dtype_list.append((field, np.uint32))
                        elif size == 2:
                            dtype_list.append((field, np.uint16))
                        elif size == 1:
                            dtype_list.append((field, np.uint8))
                    elif typ == 'I':
                        if size == 4:
                            dtype_list.append((field, np.int32))
                        elif size == 2:
                            dtype_list.append((field, np.int16))
                
                dtype = np.dtype(dtype_list)
                data = np.frombuffer(f.read(), dtype=dtype, count=num_points)
                points = np.column_stack([data['x'], data['y'], data['z']])
                return points
            else:
                # ASCII format - need to re-open as text
                is_binary = False
        
        # For ASCII, re-open as text file
        if not is_binary:
            points = []
            with open(pcd_path, 'r', encoding='utf-8') as f:
                in_data = False
                for line in f:
                    line = line.strip()
                    if line.startswith('DATA'):
                        in_data = True
                        continue
                    if in_data and line:
                        parts = line.split()
                        if len(parts) >= 3:
                            try:
                                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                            except ValueError:
                                continue
            points = np.array(points) if points else np.zeros((0, 3))
            return points
    
    except Exception as e:
        print(f"[ERROR] Failed to parse PCD: {e}")
        return np.zeros((0, 3))


def rodrigues_to_matrix(rodrigues_vec):
    """Convert Rodrigues vector [tx, ty, tz, rx, ry, rz] to 4x4 transformation matrix."""
    tvec = np.array(rodrigues_vec[:3], dtype=np.float64)
    rvec = np.array(rodrigues_vec[3:6], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


class VADASFisheyeProjector:
    """VADAS Fisheye Camera Projector for v4 calibration."""
    
    def __init__(self, intrinsic, extrinsic, lidar_to_world, image_size, direct_lidar_to_cam=None):
        # Parse intrinsic parameters
        self.k = intrinsic[0:7]  # Polynomial coefficients (u2d)
        self.s = intrinsic[7]    # Focal scale
        self.div = intrinsic[8]  # Normalization divisor
        self.ux = intrinsic[9]   # Principal point X offset
        self.uy = intrinsic[10]  # Principal point Y offset
        
        self.image_size = image_size
        
        # Build transformation matrices
        if direct_lidar_to_cam is not None:
            # 직접 제공된 LiDAR → Camera 매트릭스 사용
            self.lidar_to_camera = direct_lidar_to_cam
            print(f"[INFO] Using DIRECT LiDAR → Camera matrix")
        else:
            # 기존 방식: Extrinsic @ LiDAR_to_World
            self.extrinsic_matrix = rodrigues_to_matrix(extrinsic)
            self.lidar_to_world = lidar_to_world
            self.lidar_to_camera = self.extrinsic_matrix @ self.lidar_to_world
            print(f"[INFO] Using computed LiDAR → Camera (Extrinsic @ LiDAR_to_World)")
        
        print(f"[INFO] VADAS Fisheye Projector initialized")
        print(f"  - Image size: {image_size}")
        print(f"  - s (focal scale): {self.s}")
        print(f"  - div: {self.div}")
        print(f"  - ux, uy: ({self.ux}, {self.uy})")
        print(f"  - LiDAR → Camera Matrix:")
        for row in self.lidar_to_camera:
            print(f"    [{row[0]:10.6f}, {row[1]:10.6f}, {row[2]:10.6f}, {row[3]:10.6f}]")
    
    def _poly_eval(self, coeffs, x):
        """Evaluate polynomial using Horner's method."""
        result = 0.0
        for c in reversed(coeffs):
            result = result * x + c
        return result
    
    def project_point(self, Xc, Yc, Zc):
        """Project 3D camera coordinate to 2D image coordinate."""
        # VADAS convention: nx = -Yc, ny = -Zc
        nx = -Yc
        ny = -Zc
        dist = math.hypot(nx, ny)
        
        if dist < sys.float_info.epsilon:
            return None, None, False
        
        cosPhi = nx / dist
        sinPhi = ny / dist
        
        # Angle calculation
        theta = math.atan2(dist, Xc)
        
        # Only forward points (Xc > 0 means in front of camera)
        if Xc < 0:
            return None, None, False
        
        # Polynomial input
        xd = theta * self.s
        
        # Polynomial evaluation
        if abs(self.div) < 1e-9:
            return None, None, False
        
        rd = self._poly_eval(self.k, xd) / self.div
        
        if math.isinf(rd) or math.isnan(rd):
            return None, None, False
        
        # Image coordinates
        img_w_half = self.image_size[0] / 2
        img_h_half = self.image_size[1] / 2
        
        u = rd * cosPhi + self.ux + img_w_half
        v = rd * sinPhi + self.uy + img_h_half
        
        # Bounds check
        if 0 <= u < self.image_size[0] and 0 <= v < self.image_size[1]:
            return int(u), int(v), True
        
        return None, None, False
    
    def project_points(self, points_lidar):
        """Project LiDAR points to image coordinates."""
        if points_lidar.size == 0:
            return [], [], []
        
        # Transform to camera coordinates
        points_hom = np.hstack([points_lidar, np.ones((points_lidar.shape[0], 1))])
        points_cam = (self.lidar_to_camera @ points_hom.T).T[:, :3]
        
        projected_u = []
        projected_v = []
        depths = []
        
        for i, (Xc, Yc, Zc) in enumerate(points_cam):
            u, v, valid = self.project_point(Xc, Yc, Zc)
            if valid:
                projected_u.append(u)
                projected_v.append(v)
                depths.append(Xc)  # Depth = forward distance (Xc)
        
        return projected_u, projected_v, depths


def create_depth_overlay(rgb_image, us, vs, depths, point_size=2, max_depth=15.0):
    """Create RGB image with depth points overlaid."""
    overlay = rgb_image.copy()
    
    if len(us) == 0:
        return overlay
    
    # Normalize depths for colormap
    depths_arr = np.array(depths)
    depths_normalized = np.clip(depths_arr / max_depth, 0, 1)
    depths_uint8 = (depths_normalized * 255).astype(np.uint8)
    
    # Apply JET colormap
    depth_colors = cv2.applyColorMap(depths_uint8.reshape(-1, 1, 1), cv2.COLORMAP_JET)
    depth_colors = depth_colors.reshape(-1, 3)
    
    # Draw points
    for u, v, color in zip(us, vs, depth_colors):
        cv2.circle(overlay, (u, v), point_size, color.tolist(), -1)
    
    return overlay


def verify_single_frame(data_dir: Path, frame_id: str, output_dir: Path = None):
    """Verify calibration for a single frame."""
    image_path = data_dir / "image_a6" / f"{frame_id}.jpg"
    pcd_path = data_dir / "pcd" / f"{frame_id}.pcd"
    
    if not image_path.exists():
        print(f"[ERROR] Image not found: {image_path}")
        return None
    if not pcd_path.exists():
        print(f"[ERROR] PCD not found: {pcd_path}")
        return None
    
    print(f"\n{'='*60}")
    print(f"Verifying Frame: {frame_id}")
    print(f"{'='*60}")
    
    # Load image
    rgb_image = cv2.imread(str(image_path))
    if rgb_image is None:
        print(f"[ERROR] Failed to load image: {image_path}")
        return None
    
    print(f"[INFO] Image size: {rgb_image.shape[1]}x{rgb_image.shape[0]}")
    
    # Load PCD
    points = parse_pcd_file(pcd_path)
    print(f"[INFO] Loaded {len(points)} points from PCD")
    
    # Initialize projector
    calib = DEFAULT_CALIB_v4["a6"]
    
    # 직접 LiDAR to Camera 매트릭스 사용 여부에 따라 projector 초기화
    if USE_DIRECT_LIDAR_TO_CAM:
        projector = VADASFisheyeProjector(
            intrinsic=calib["intrinsic"],
            extrinsic=calib["extrinsic"],
            lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v4,
            image_size=calib["image_size"],
            direct_lidar_to_cam=DEFAULT_LIDAR_TO_CAM_v4
        )
    else:
        projector = VADASFisheyeProjector(
            intrinsic=calib["intrinsic"],
            extrinsic=calib["extrinsic"],
            lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v4,
            image_size=calib["image_size"]
        )
    
    # Filter forward points only (X > 0 in v4 coordinate system)
    forward_mask = points[:, 0] > 0
    forward_points = points[forward_mask]
    print(f"[INFO] Forward points (X > 0): {len(forward_points)}")
    
    # Project points
    us, vs, depths = projector.project_points(forward_points)
    print(f"[INFO] Successfully projected: {len(us)} points")
    
    if len(us) > 0:
        print(f"[INFO] Depth range: {min(depths):.2f}m ~ {max(depths):.2f}m")
    
    # Create visualization
    overlay = create_depth_overlay(rgb_image, us, vs, depths, point_size=3, max_depth=15.0)
    
    # Add info text
    cv2.putText(overlay, f"Frame: {frame_id}", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(overlay, f"Points: {len(us)}/{len(forward_points)}", (20, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(overlay, f"v4 Calibration (Indoor)", (20, 120), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    
    # Add LiDAR → Camera matrix info
    cv2.putText(overlay, "LiDAR to Camera Matrix:", (20, 170), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    for i, row in enumerate(projector.lidar_to_camera):
        matrix_text = f"[{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}, {row[3]:8.5f}]"
        cv2.putText(overlay, matrix_text, (20, 200 + i * 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
    
    # Save result
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{frame_id}_v4_verification.jpg"
        cv2.imwrite(str(output_path), overlay)
        print(f"[SAVE] {output_path}")
    
    return overlay


def verify_multiple_frames(data_dir: Path, num_frames: int = 5, output_dir: Path = None):
    """Verify calibration for multiple random frames."""
    pcd_dir = data_dir / "pcd"
    pcd_files = sorted(list(pcd_dir.glob("*.pcd")))
    
    if len(pcd_files) == 0:
        print("[ERROR] No PCD files found")
        return
    
    # Select random frames
    if num_frames > len(pcd_files):
        num_frames = len(pcd_files)
    
    selected_files = random.sample(pcd_files, num_frames)
    
    print(f"\n{'='*60}")
    print(f"Verifying {num_frames} random frames from {len(pcd_files)} total")
    print(f"{'='*60}")
    
    results = []
    for pcd_file in selected_files:
        frame_id = pcd_file.stem
        result = verify_single_frame(data_dir, frame_id, output_dir)
        if result is not None:
            results.append((frame_id, result))
    
    # Create grid visualization if multiple frames
    if len(results) > 1 and output_dir:
        create_comparison_grid(results, output_dir)
    
    return results


def create_comparison_grid(results, output_dir):
    """Create a grid of all verification results."""
    if not results:
        return
    
    # Resize all images to same size for grid
    target_h, target_w = 384, 480  # Smaller for grid
    
    resized = []
    for frame_id, img in results:
        img_resized = cv2.resize(img, (target_w, target_h))
        # Add frame ID
        cv2.putText(img_resized, frame_id, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        resized.append(img_resized)
    
    # Create grid
    n = len(resized)
    cols = min(3, n)
    rows = (n + cols - 1) // cols
    
    grid = np.zeros((rows * target_h, cols * target_w, 3), dtype=np.uint8)
    
    for i, img in enumerate(resized):
        r = i // cols
        c = i % cols
        grid[r*target_h:(r+1)*target_h, c*target_w:(c+1)*target_w] = img
    
    output_path = output_dir / "v4_verification_grid.jpg"
    cv2.imwrite(str(output_path), grid)
    print(f"\n[SAVE] Comparison grid: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Indoor Dataset v4 Calibration Verification")
    parser.add_argument("--data_dir", type=str, 
                        default=r"D:\data\ncdb-cls\ncdb-cls-indoor",
                        help="Path to indoor dataset")
    parser.add_argument("--frame", type=str, default=None,
                        help="Specific frame ID to verify (e.g., 0000020100)")
    parser.add_argument("--frames", type=str, default=None,
                        help="Comma-separated frame IDs to verify and build a grid (e.g., 0000020100,0000020200)")
    parser.add_argument("--random", type=int, default=5,
                        help="Number of random frames to verify")
    parser.add_argument("--output_dir", type=str, default="./output/indoor_verification",
                        help="Output directory for results")
    
    args = parser.parse_args()
    
    data_dir = Path(args.data_dir)
    output_dir = Path(args.output_dir)
    
    if not data_dir.exists():
        print(f"[ERROR] Data directory not found: {data_dir}")
        return
    
    print(f"\n{'='*60}")
    print(f"Indoor Dataset v4 Calibration Verification")
    print(f"{'='*60}")
    print(f"Data directory: {data_dir}")
    print(f"Output directory: {output_dir}")
    
    if args.frames:
        # Use explicit comma-separated frames
        frame_ids = [f.strip() for f in args.frames.split(',') if f.strip()]
        if not frame_ids:
            print("[ERROR] --frames provided but no valid IDs parsed")
            return
        results = []
        for fid in frame_ids:
            if fid.isdigit() and len(fid) < 10:
                fid = fid.zfill(10)
            res = verify_single_frame(data_dir, fid, output_dir)
            if res is not None:
                results.append((fid, res))
        if len(results) > 1:
            create_comparison_grid(results, output_dir)
    elif args.frame:
        # Verify specific frame
        verify_single_frame(data_dir, args.frame, output_dir)
    else:
        # Verify random frames
        verify_multiple_frames(data_dir, args.random, output_dir)
    
    print(f"\n{'='*60}")
    print(f"Verification complete!")
    print(f"Check results in: {output_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
