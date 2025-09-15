import sys
import os
import json
import math
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
from tqdm import tqdm

import numpy as np
import cv2
import matplotlib.pyplot as plt

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Warning: open3d not found. Some features will be disabled.", file=sys.stderr)

# =============================================================================
# Constants for c_circles generation (ported from find_road_by_geometry.py)
# =============================================================================
BLUE_DEPTH_Z = -0.771
NUM_C_RADII = 50
CIRCLE_SEGS = 512
SKIP_FAR_COUNT = 1
NEAR_BIAS = 1.2
MIN_FIRST_RADIUS = 0.01
RADIUS_DISTRIBUTION = "uniform"  # 'uniform' | 'near' | 'cosine'
XY_MIN_SEPARATION = 0.1

# =============================================================================
# Road Geometry Analysis (from find_road_by_geometry.py)
# =============================================================================
def find_nearest_road_point_and_generate_synthetic_pcd(
    pcd_path: Path,
    ground_z_min: float = -3.0,
    ground_z_max: float = 0.0,
    min_xy_distance_from_origin: float = 2.0,
    xy_radius_threshold: float = 10.0,
    y_min: Optional[float] = None,
    y_max: Optional[float] = None,
    num_radius_divisions: int = 20,
    points_per_circle: int = 200,
    keep_original_points: bool = True,
    exclude_outermost_circle: bool = True  # kept for CLI compatibility, unused now
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    REPLACED: Generate PCD using closest-line-by-angle algorithm (3D Euclidean distance) from find_road_by_geometry.py.
    Returns: (closest_line_points, original_points)
    """
    try:
        # Parameters matching find_road_by_geometry.py behavior
        exclude_positive_x = True
        angular_resolution = 1.0  # degrees per bin
        y_zero_band = 0.01        # for debug print only

        print(f"[DEBUG] (Closest-Line) called with:")
        print(f"  - ground_z_min/max: [{ground_z_min}, {ground_z_max}]")
        print(f"  - min_xy_distance_from_origin: {min_xy_distance_from_origin}")
        print(f"  - xy_radius_threshold: {xy_radius_threshold}")
        print(f"  - exclude_positive_x: {exclude_positive_x}")
        print(f"  - angular_resolution: {angular_resolution} deg")

        # Step 1: Load PCD
        if OPEN3D_AVAILABLE:
            pcd = o3d.io.read_point_cloud(str(pcd_path))
            if not pcd.has_points():
                print(f"Warning: No points found in {pcd_path}")
                return np.empty((0, 3)), None
            all_points_np = np.asarray(pcd.points)
        else:
            # Fallback ASCII parser (simple)
            points = []
            with open(pcd_path, 'r', encoding='utf-8') as f:
                data_started = False
                for line in f:
                    if data_started:
                        try:
                            parts = line.strip().split()
                            if len(parts) >= 3:
                                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                        except (ValueError, IndexError):
                            continue
                    elif line.startswith("DATA ascii"):
                        data_started = True
            all_points_np = np.array(points, dtype=np.float32) if points else np.empty((0, 3), dtype=np.float32)

        if all_points_np.size == 0:
            print(f"Warning: Empty point cloud in {pcd_path}")
            return np.empty((0, 3)), None

        # === NEW: Early exclusion of front strip (applies ONLY to original cloud) ===
        strip_mask = (
            # (all_points_np[:,1] >= -0.7) &
            # (all_points_np[:,1] <= 0.5) &
            (all_points_np[:,0] >= 0.0)
        )
        if np.any(strip_mask):
            removed = int(strip_mask.sum())
            all_points_np = all_points_np[~strip_mask]
            print(f"[STRIP] Removed {removed} front-strip points at load (remain {all_points_np.shape[0]})")

        # Step 2: Ground Z filter (strict inequalities to match reference)
        ground_indices = np.where(
            (all_points_np[:, 2] > ground_z_min) & (all_points_np[:, 2] < ground_z_max)
        )[0]
        if len(ground_indices) == 0:
            print(f"Error: No ground candidate points in Z-range ({ground_z_min},{ground_z_max}).")
            return np.empty((0, 3)), all_points_np

        ground_points = all_points_np[ground_indices]

        # Step 2b: XY min distance filter (> threshold)
        if min_xy_distance_from_origin > 0.0:
            xy_dist = np.linalg.norm(ground_points[:, :2], axis=1)
            keep_idx = np.where(xy_dist > min_xy_distance_from_origin)[0]
            if len(keep_idx) == 0:
                print(f"Error: No ground candidates after min XY distance filter ({min_xy_distance_from_origin}).")
                return np.empty((0, 3)), all_points_np
            ground_points = ground_points[keep_idx]
            ground_indices = ground_indices[keep_idx]

        # Step 2c: XY radius threshold filter (< threshold)
        if np.isfinite(xy_radius_threshold):
            xy_dist = np.linalg.norm(ground_points[:, :2], axis=1)
            keep_idx = np.where(xy_dist < xy_radius_threshold)[0]
            if len(keep_idx) == 0:
                print(f"Error: No ground candidates after XY radius threshold filter ({xy_radius_threshold}).")
                return np.empty((0, 3)), all_points_np
            ground_points = ground_points[keep_idx]
            ground_indices = ground_indices[keep_idx]

        # Step 2d: Exclude positive X (x <= 0 only)
        if exclude_positive_x:
            keep_idx = np.where(ground_points[:, 0] <= 0.0)[0]
            if len(keep_idx) == 0:
                print("Error: No ground candidates after excluding positive X.")
                return np.empty((0, 3)), all_points_np
            ground_points = ground_points[keep_idx]
            ground_indices = ground_indices[keep_idx]

        # Step 3: Closest per azimuth bin by 3D Euclidean distance
        num_angular_bins = int(360.0 / angular_resolution)
        min_dist_per_bin = np.full(num_angular_bins, np.inf, dtype=np.float64)
        closest_point_idx_per_bin = np.full(num_angular_bins, -1, dtype=np.int32)

        angles = np.arctan2(ground_points[:, 1], ground_points[:, 0])
        # Match binning from reference (uses (num_bins - 1))
        bin_indices = ((angles + np.pi) / (2 * np.pi) * (num_angular_bins - 1)).astype(int)
        dists_3d = np.linalg.norm(ground_points, axis=1)

        for i in range(ground_points.shape[0]):
            b = bin_indices[i]
            d = dists_3d[i]
            if d < min_dist_per_bin[b]:
                min_dist_per_bin[b] = d
                closest_point_idx_per_bin[b] = i

        valid_bins = [b for b in range(num_angular_bins) if closest_point_idx_per_bin[b] != -1]
        if len(valid_bins) == 0:
            print("Error: Could not determine closest line points from ground candidates.")
            return np.empty((0, 3)), all_points_np

        valid_indices_in_ground = closest_point_idx_per_bin[valid_bins]
        closest_line_points = ground_points[valid_indices_in_ground]

        # Step 4: Debug print for a single y≈0 point (from all original points), matching reference
        x_vals = all_points_np[:, 0]
        y_vals = all_points_np[:, 1]
        y_zero_mask = (np.abs(y_vals) <= y_zero_band) & (x_vals <= 0.0)
        y_zero_indices = np.where(y_zero_mask)[0]
        if y_zero_indices.size > 0:
            cand_points = all_points_np[y_zero_indices]
            dists_all = np.linalg.norm(cand_points, axis=1)
            pos_mask = dists_all > 0.0
            if np.any(pos_mask):
                cand_indices = y_zero_indices[pos_mask]
                dists_pos = dists_all[pos_mask]
                pts_pos = cand_points[pos_mask]
                best_local = int(np.argmin(dists_pos))
                selected_idx = int(cand_indices[best_local])
                selected_point = pts_pos[best_local]
                selected_distance = float(dists_pos[best_local])
                selected_angle_deg = float(np.degrees(np.arctan2(selected_point[1], selected_point[0])))
                selected_dict = {
                    "x": float(selected_point[0]),
                    "y": float(selected_point[1]),
                    "z": float(selected_point[2]),
                }
                print(f"[y≈0 one-point] idx={selected_idx}, xyz={selected_dict}, dist={selected_distance:.4f} m, angle_deg={selected_angle_deg:.2f}")
            else:
                print("[y≈0 one-point] no candidate with positive distance")
        else:
            print("[y≈0 one-point] no candidates within |y| band and x<=0")

        print("-" * 30)
        print(f"Analysis Complete for {pcd_path.name}")
        print(f"Method: 3D Euclidean Distance per {angular_resolution}-degree angle")
        print(f"Found {len(closest_line_points)} points for the closest line after filtering.")
        print("-" * 30)

        # Return closest line points and original points for optional merging
        return closest_line_points, all_points_np

    except Exception as e:
        print(f"Error processing {pcd_path}: {e}")
        return np.empty((0, 3)), None

def build_radii_along_c_vals(c_len: float) -> np.ndarray:
    """Radii along |c| with optional near/cosine bias and skipping far rings."""
    if c_len <= 0 or NUM_C_RADII <= 0:
        return np.zeros((0,), dtype=np.float64)
    t = np.linspace(0.0, 1.0, NUM_C_RADII + 1, dtype=np.float64)[1:]
    if RADIUS_DISTRIBUTION == "near":
        if abs(NEAR_BIAS - 1.0) > 1e-9:
            t = t ** NEAR_BIAS
    elif RADIUS_DISTRIBUTION == "cosine":
        t = (1.0 - np.cos(np.pi * t)) * 0.5
    radii = t * c_len
    if MIN_FIRST_RADIUS > 0.0:
        radii = radii[radii >= MIN_FIRST_RADIUS]
    if SKIP_FAR_COUNT > 0:
        keep = max(len(radii) - int(SKIP_FAR_COUNT), 0)
        radii = radii[:keep]
    return radii


def build_c_circles_points_from_cloud(
    all_points_np: np.ndarray,
    y_zero_band: float = 0.01,
    blue_depth_z: float = BLUE_DEPTH_Z,
    num_c_radii: int = NUM_C_RADII,
    circle_segs: int = CIRCLE_SEGS,
    skip_far_count: int = SKIP_FAR_COUNT,
    min_first_radius: float = MIN_FIRST_RADIUS,
    xy_min_separation: float = XY_MIN_SEPARATION,
    radius_distribution: str = RADIUS_DISTRIBUTION,
    near_bias: float = NEAR_BIAS,
) -> np.ndarray:
    """
    Generate c_circles PCD points (black rings) identical in logic to find_road_by_geometry.py:
    - Select 'purple' point: closest 3D by Euclidean distance among |y|<=band and x<=0.
    - Define tilted basis using c = purple - (0,0,blue_depth_z).
    - Build rings with radii along c; keep only X<=0 segment.
    - Optionally filter ring points too close in XY to original cloud using KDTree.
    Returns N x 3 array of ring points.
    """
    if all_points_np is None or all_points_np.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    pts = all_points_np.astype(np.float64, copy=False)

    # 1) Select purple point (|y|<=band and x<=0) with minimal Euclidean distance (>0)
    x_vals = pts[:, 0]
    y_vals = pts[:, 1]
    mask = (np.abs(y_vals) <= y_zero_band) & (x_vals <= 0.0)
    idxs = np.where(mask)[0]
    if idxs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    cand = pts[idxs]
    dists = np.linalg.norm(cand, axis=1)
    pos_mask = dists > 0.0
    if not np.any(pos_mask):
        return np.zeros((0, 3), dtype=np.float64)

    cand_idxs = idxs[pos_mask]
    dists_pos = dists[pos_mask]
    pts_pos = cand[pos_mask]
    best_local = int(np.argmin(dists_pos))
    purple = pts_pos[best_local]

    # 2) Build tilted basis aligned to purple azimuth and c slope
    x_p, y_p, z_p = float(purple[0]), float(purple[1]), float(purple[2])
    b_len = float(math.hypot(x_p, y_p))
    center = np.array([0.0, 0.0, blue_depth_z], dtype=np.float64)
    c_vec = purple - center
    c_len = float(np.linalg.norm(c_vec))

    eps = 1e-9
    u_b_xy = np.array([x_p, y_p, 0.0], dtype=np.float64)
    n_b = np.linalg.norm(u_b_xy)
    if n_b <= eps:
        u_b_xy = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        u_b_xy /= n_b

    z_hat = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    u_tan = np.cross(z_hat, u_b_xy)

    slope_c = float((z_p - blue_depth_z) / max(b_len, eps))
    u_elev = u_b_xy + slope_c * z_hat
    u_elev /= max(np.linalg.norm(u_elev), eps)

    # Radii along c (match helper defaults)
    # overwrite globals conditionally from parameters
    global NUM_C_RADII, CIRCLE_SEGS, SKIP_FAR_COUNT, MIN_FIRST_RADIUS, RADIUS_DISTRIBUTION, NEAR_BIAS
    NUM_C_RADII = num_c_radii
    CIRCLE_SEGS = circle_segs
    SKIP_FAR_COUNT = skip_far_count
    MIN_FIRST_RADIUS = min_first_radius
    RADIUS_DISTRIBUTION = radius_distribution
    NEAR_BIAS = near_bias

    radii = build_radii_along_c_vals(c_len)
    if radii.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    # KD-tree on original XY (embedded at z=0)
    kdtree = None
    if OPEN3D_AVAILABLE and xy_min_separation > 0.0:
        try:
            orig_xy = np.column_stack([pts[:, 0], pts[:, 1], np.zeros((pts.shape[0],), dtype=np.float64)])
            kd_pcd = o3d.geometry.PointCloud()
            kd_pcd.points = o3d.utility.Vector3dVector(orig_xy)
            kdtree = o3d.geometry.KDTreeFlann(kd_pcd)
        except Exception:
            kdtree = None

    kept_points = []
    ts = np.linspace(0.0, 2.0 * np.pi, CIRCLE_SEGS, dtype=np.float64)
    cos_t = np.cos(ts)[:, None]
    sin_t = np.sin(ts)[:, None]

    for r in radii:
        # Skip ring through purple point
        if abs(r - c_len) <= max(1e-9, 1e-6 * c_len):
            continue

        ring = center + r * (cos_t * u_elev + sin_t * u_tan)
        # Keep only X<=0
        ring = ring[ring[:, 0] <= 0.0]
        if ring.size == 0:
            continue

        if kdtree is None or xy_min_separation <= 0.0:
            kept_points.append(ring)
            continue

        # Filter out points too close in XY to original cloud
        local_kept = []
        for p in ring:
            q = np.array([p[0], p[1], 0.0], dtype=np.float64)
            try:
                k, _, _ = kdtree.search_radius_vector_3d(q, float(xy_min_separation))
                if k == 0:
                    local_kept.append(p)
            except Exception:
                local_kept.append(p)
        if local_kept:
            kept_points.append(np.asarray(local_kept, dtype=np.float64))

    if not kept_points:
        return np.zeros((0, 3), dtype=np.float64)

    return np.vstack(kept_points).astype(np.float64)

# =============================================================================
# Camera Models and Depth Map Generation (from create_depth_maps.py)
# =============================================================================
class CameraModelBase:
    """Base class for camera projection models."""
    def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
        raise NotImplementedError

class VADASFisheyeCameraModel(CameraModelBase):
    """VADAS Polynomial Fisheye Camera Model, assuming +X is forward."""
    def __init__(self, intrinsic: List[float], image_size: Optional[Tuple[int, int]] = None):
        if len(intrinsic) < 11:
            raise ValueError("VADAS intrinsic must have at least 11 parameters.")
        self.k = intrinsic[0:7]
        self.s = intrinsic[7]
        self.div = intrinsic[8]
        self.ux = intrinsic[9]
        self.uy = intrinsic[10]
        self.image_size = image_size

    def _poly_eval(self, coeffs: List[float], x: float) -> float:
        res = 0.0
        for c in reversed(coeffs):
            res = res * x + c
        return res

    def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
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

        xd = theta * self.s
        if abs(self.div) < 1e-9:
            return 0, 0, False
        
        rd = self._poly_eval(self.k, xd) / self.div
        if math.isinf(rd) or math.isnan(rd):
            return 0, 0, False

        img_w_half = (self.image_size[0] / 2) if self.image_size else 0
        img_h_half = (self.image_size[1] / 2) if self.image_size else 0

        u = rd * cosPhi + self.ux + img_w_half
        v = rd * sinPhi + self.uy + img_h_half
        
        return int(round(u)), int(round(v)), True

class SensorInfo:
    """Holds camera sensor information."""
    def __init__(self, name: str, model: CameraModelBase, intrinsic: List[float], extrinsic: np.ndarray, image_size: Optional[Tuple[int, int]] = None):
        self.name = name
        self.model = model
        self.intrinsic = intrinsic
        self.extrinsic = extrinsic
        self.image_size = image_size

class CalibrationDB:
    """Manages camera calibration data."""
    def __init__(self, calib_dict: Dict[str, Any], lidar_to_world: Optional[np.ndarray] = None):
        self.sensors: Dict[str, SensorInfo] = {}
        self.lidar_to_world = lidar_to_world if lidar_to_world is not None else np.eye(4)

        for cam_name, calib_data in calib_dict.items():
            model_type = calib_data["model"]
            intrinsic = calib_data["intrinsic"]
            extrinsic_raw = calib_data["extrinsic"]
            image_size = tuple(calib_data["image_size"]) if "image_size" in calib_data and calib_data["image_size"] else None

            extrinsic_matrix = self._rodrigues_to_matrix(extrinsic_raw) if len(extrinsic_raw) == 6 else np.array(extrinsic_raw).reshape(4, 4)

            if model_type == "vadas":
                camera_model = VADASFisheyeCameraModel(intrinsic, image_size=image_size)
            else:
                raise ValueError(f"Unsupported camera model: {model_type}.")
            
            self.sensors[cam_name] = SensorInfo(cam_name, camera_model, intrinsic, extrinsic_matrix, image_size)

    def _rodrigues_to_matrix(self, rvec_tvec: List[float]) -> np.ndarray:
        tvec = np.array(rvec_tvec[0:3]).reshape(3, 1)
        rvec = np.array(rvec_tvec[3:6])
        R, _ = cv2.Rodrigues(rvec)
        
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 0:3] = R
        transform_matrix[0:3, 3:4] = tvec
        return transform_matrix

    def get(self, name: str) -> SensorInfo:
        if name not in self.sensors:
            raise ValueError(f"Sensor '{name}' not found in calibration database.")
        return self.sensors[name]

class LidarCameraProjector:
    """Projects LiDAR point clouds to create depth maps."""
    def __init__(self, calib_db: CalibrationDB):
        self.calib_db = calib_db

    def project_cloud_to_depth_map(self, sensor_name: str, cloud_xyz: np.ndarray, image_size: Tuple[int, int]) -> Optional[np.ndarray]:
        sensor_info = self.calib_db.get(sensor_name)
        camera_model = sensor_info.model
        cam_extrinsic = sensor_info.extrinsic
        image_width, image_height = image_size

        if isinstance(camera_model, VADASFisheyeCameraModel) and camera_model.image_size is None:
            camera_model.image_size = (image_width, image_height)

        depth_map = np.zeros((image_height, image_width), dtype=np.float32)
        
        cloud_xyz_hom = np.hstack((cloud_xyz, np.ones((cloud_xyz.shape[0], 1))))
        
        # [FIX] 포인트 필터링 추가 - create_depth_maps.py와 동일
        # Define the exclusion condition based on Y and X coordinates
        # Exclude points where (Y <= 0.5 and Y >= -0.7) AND (X >= 0.0)
        exclude_y_condition = (cloud_xyz_hom[:, 1] <= 0.5) & (cloud_xyz_hom[:, 1] >= -0.7)
        exclude_x_condition = (cloud_xyz_hom[:, 0] >= 0.0)
        
        # Combine conditions to get points to EXCLUDE
        points_to_exclude = exclude_y_condition & exclude_x_condition
        
        # Keep only the points that are NOT in the exclusion set
        cloud_xyz_hom = cloud_xyz_hom[~points_to_exclude]
        
        # [FIX] 올바른 좌표계 변환 - create_depth_maps.py와 동일
        lidar_to_camera_transform = cam_extrinsic @ self.calib_db.lidar_to_world
        points_cam_hom = (lidar_to_camera_transform @ cloud_xyz_hom.T).T
        points_cam = points_cam_hom[:, :3]

        for i in range(points_cam.shape[0]):
            Xc, Yc, Zc = points_cam[i]
            
            if Xc <= 0:
                continue

            u, v, valid_projection = camera_model.project_point(Xc, Yc, Zc)

            if valid_projection and 0 <= u < image_width and 0 <= v < image_height:
                # Occlusion check
                if depth_map[v, u] == 0 or depth_map[v, u] > Xc:
                    depth_map[v, u] = Xc
        
        return depth_map

    def project_cloud_to_depth_map_with_labels(
        self,
        sensor_name: str,
        cloud_xyz: np.ndarray,
        labels: np.ndarray,  # 0=original, 1=synthetic
        image_size: Tuple[int, int]
    ) -> Tuple[np.ndarray, np.ndarray]:
        sensor_info = self.calib_db.get(sensor_name)
        camera_model = sensor_info.model
        cam_extrinsic = sensor_info.extrinsic
        w, h = image_size

        if isinstance(camera_model, VADASFisheyeCameraModel) and camera_model.image_size is None:
            camera_model.image_size = (w, h)

        depth = np.zeros((h, w), dtype=np.float32)
        provenance = np.full((h, w), -1, dtype=np.int8)  # -1: empty, 0 orig, 1 synth

        pts_h = np.hstack([cloud_xyz, np.ones((cloud_xyz.shape[0], 1))])
        T = cam_extrinsic @ self.calib_db.lidar_to_world
        cam_pts = (T @ pts_h.T).T[:, :3]

        for i, (Xc, Yc, Zc) in enumerate(cam_pts):
            if Xc <= 0:
                continue
            u, v, ok = camera_model.project_point(Xc, Yc, Zc)
            if not ok or not (0 <= u < w and 0 <= v < h):
                continue
            # Occlusion: nearer Xc wins
            if depth[v, u] == 0 or depth[v, u] > Xc:
                depth[v, u] = Xc
                provenance[v, u] = labels[i]

        return depth, provenance

# =============================================================================
# Utility Functions
# =============================================================================
def save_synthetic_pcd(points: np.ndarray, output_path: Path) -> None:
    """Saves point cloud data to a PCD file."""
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        if OPEN3D_AVAILABLE:
            # Open3D를 사용한 저장
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(str(output_path), pcd, write_ascii=True)
        else:
            # 수동으로 ASCII PCD 파일 작성
            with open(output_path, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                
                for point in points:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
                    
        print(f"[SAVE] PCD saved ({len(points):,} points): {output_path}")
        
    except Exception as e:
        print(f"Error saving PCD to {output_path}: {e}")
        raise

def save_depth_map(path: Path, depth_map: np.ndarray) -> None:
    """Saves a depth map as a 16-bit PNG image, following KITTI conventions."""
    depth_map_uint16 = (depth_map * 256.0).astype(np.uint16)
    
    # [FIX] OpenCV 버전 호환성 처리
    try:
        # 최신 OpenCV에서 16비트 PNG 명시적 저장 시도
        cv2.imwrite(str(path), depth_map_uint16, [cv2.IMWRITE_PNG_BIT_DEPTH, 16])
    except AttributeError:
        # IMWRITE_PNG_BIT_DEPTH가 없는 경우, 기본 저장 (16비트는 자동으로 감지됨)
        print(f"Warning: cv2.IMWRITE_PNG_BIT_DEPTH not available. Using default PNG saving for {path.name}")
        cv2.imwrite(str(path), depth_map_uint16)
    except Exception as e:
        # 다른 오류가 발생한 경우 기본 저장으로 fallback
        print(f"Warning: PNG saving with bit depth failed ({e}). Using default saving for {path.name}")
        cv2.imwrite(str(path), depth_map_uint16)

def create_depth_visualization(depth_map: np.ndarray, output_path: Path, title: str) -> None:
    """Creates and saves a depth map visualization with statistics."""
    try:
        import matplotlib.pyplot as plt
        
        valid_depths = depth_map[depth_map > 0]
        if len(valid_depths) == 0:
            print(f"Warning: No valid depth values found for visualization: {output_path.name}")
            return
            
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 깊이 맵 시각화
        im1 = ax1.imshow(depth_map, cmap='magma', vmin=0, vmax=np.percentile(valid_depths, 95))
        masked = np.ma.masked_where(depth_map == 0, depth_map)
        cmap = plt.cm.get_cmap('magma').copy()
        cmap.set_bad(color='white')
        im1 = ax1.imshow(masked, cmap=cmap, vmin=0, vmax=np.percentile(valid_depths, 95))
        plt.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04, label='Depth (m)')
        
        # 히스토그램
        ax2.hist(valid_depths, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax2.set_xlabel('Depth (m)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Depth Distribution')
        ax2.grid(True, alpha=0.3)
        
        # 통계 텍스트 추가
        stats_text = f"Stats:\nMin: {valid_depths.min():.2f}m\nMax: {valid_depths.max():.2f}m\nMean: {valid_depths.mean():.2f}m\nStd: {valid_depths.std():.2f}m"
        ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"[SAVE] Visualization saved: {output_path}")
        
    except ImportError:
        print("Warning: matplotlib not available. Skipping visualization.")
    except Exception as e:
        print(f"Error creating visualization for {output_path}: {e}")

def create_depth_colormap_image(depth_map: np.ndarray, output_path: Path) -> None:
    """Creates and saves a colorized depth map image using OpenCV."""
    try:
        valid_depths = depth_map[depth_map > 0]
        if len(valid_depths) == 0:
            print(f"Warning: No valid depth values for colormap: {output_path.name}")
            return
        
        # [DEBUG] 깊이 값 범위 출력
        min_depth = np.min(valid_depths)
        max_depth = np.max(valid_depths)
        mean_depth = np.mean(valid_depths)
        percentile_50 = np.percentile(valid_depths, 50)  # 중간값
        percentile_75 = np.percentile(valid_depths, 75)
        percentile_95 = np.percentile(valid_depths, 95)
        percentile_99 = np.percentile(valid_depths, 99)
        
        print(f"[DEBUG] Depth values for {output_path.name}:")
        print(f"  - Valid pixels: {len(valid_depths):,}")
        print(f"  - Min depth: {min_depth:.3f}m")
        print(f"  - Max depth: {max_depth:.3f}m")
        print(f"  - Mean depth: {mean_depth:.3f}m")
        print(f"  - 50th percentile: {percentile_50:.3f}m")
        print(f"  - 75th percentile: {percentile_75:.3f}m")
        print(f"  - 95th percentile: {percentile_95:.3f}m")
        print(f"  - 99th percentile: {percentile_99:.3f}m")
        
        # [FIX] 동적 정규화 범위 선택
        # 1. 대부분의 포인트가 근거리에 있는 경우: 75th percentile 사용
        # 2. 포인트가 넓게 분포된 경우: 95th percentile 사용
        if percentile_75 <= 8.0:  # 대부분이 8m 이내
            normalization_max = percentile_75
            range_type = "75th percentile (near-field focus)"
        elif percentile_95 <= 15.0:  # 대부분이 15m 이내
            normalization_max = percentile_95 * 0.8  # 95%의 80% 사용
            range_type = "80% of 95th percentile (mid-range focus)"
        else:  # 원거리까지 넓게 분포
            normalization_max = percentile_95
            range_type = "95th percentile (full-range)"
        
        # 최소 정규화 범위 보장 (너무 작으면 3m로 설정)
        if normalization_max < 3.0:
            normalization_max = 3.0
            range_type = "minimum 3m range"
        
        print(f"  - Normalization max: {normalization_max:.3f}m ({range_type})")
        
        # [FIX] 0-255 범위로 정규화 (클리핑으로 outlier 제거)
        normalized_depth = np.clip(depth_map / normalization_max * 255, 0, 255).astype(np.uint8)
        
        # [FIX] JET 컬러맵 적용
        colored_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
        
        # 유효하지 않은 픽셀은 검은색으로 설정
        mask = depth_map == 0
        colored_depth[mask] = [255, 255, 255]
        
        # [DEBUG] 정규화 후 값 범위 출력
        normalized_valid = normalized_depth[depth_map > 0]
        near_field_pixels = np.sum((depth_map > 0) & (depth_map <= 5.0))  # 5m 이내 픽셀 수
        mid_field_pixels = np.sum((depth_map > 5.0) & (depth_map <= 15.0))  # 5-15m 픽셀 수
        far_field_pixels = np.sum(depth_map > 15.0)  # 15m 초과 픽셀 수
        
        print(f"  - Normalized range: [{np.min(normalized_valid)}, {np.max(normalized_valid)}]")
        print(f"  - Near-field (0-5m): {near_field_pixels:,} pixels")
        print(f"  - Mid-field (5-15m): {mid_field_pixels:,} pixels")
        print(f"  - Far-field (>15m): {far_field_pixels:,} pixels")
        
        # [DEBUG] JET 컬러맵에서 각 거리대별 대표 색상 안내
        print(f"  - Color mapping guide:")
        print(f"    * 0-{normalization_max*0.2:.1f}m: Deep Blue")
        print(f"    * {normalization_max*0.2:.1f}-{normalization_max*0.4:.1f}m: Cyan/Green")
        print(f"    * {normalization_max*0.4:.1f}-{normalization_max*0.6:.1f}m: Yellow")
        print(f"    * {normalization_max*0.6:.1f}-{normalization_max*0.8:.1f}m: Orange")
        print(f"    * {normalization_max*0.8:.1f}m+: Red")
        
        cv2.imwrite(str(output_path), colored_depth)
        print(f"[SAVE] Colorized depth map saved (JET colormap, {range_type}): {output_path}")
        
    except Exception as e:
        print(f"Error creating colorized depth map for {output_path}: {e}")

# =============================================================================
# Default Configuration
# =============================================================================
DEFAULT_CALIB = {
    "a6": {
        "model": "vadas",
        "intrinsic": [-0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,
                      1.0447, 0.0021, 44.9516, 2.48822, 0, 0.9965, -0.0067,
                      -0.0956, 0.1006, -0.054, 0.0106],
        "extrinsic": [0.293769, -0.0542026, -0.631615, -0.00394431, -0.33116, -0.00963617],
        "image_size": None
    }
}

DEFAULT_LIDAR_TO_WORLD = np.array([
    [-0.998752, -0.00237052, -0.0498847,  0.0375091],
    [ 0.00167658, -0.999901,   0.0139481,  0.0349093],
    [-0.0499128,  0.0138471,   0.998658,   0.771878],
    [ 0.,         0.,          0.,         1.       ]
])

# =============================================================================
# Main Pipeline
# =============================================================================
# [DEBUG] 파라미터 전달 과정을 추적하기 위한 수정
def run_integrated_pipeline(
    parent_folder: Path,
    camera_name: str = "a6",
    ground_z_min: float = -3.0,
    ground_z_max: float = 0.0,
    min_xy_distance: float = 2.0,
    xy_radius_threshold: float = 10.0,
    y_min: Optional[float] = None,
    y_max: Optional[float] = None,
    num_radius_divisions: int = 20,    # kept for CLI compatibility
    points_per_circle: int = 200,      # kept for CLI compatibility
    reference_image_size: Tuple[int, int] = (1920, 1536),
    keep_original_points: bool = True,  # kept for CLI compatibility (no effect)
    exclude_outermost_circle: bool = True  # kept for CLI compatibility (no effect)
) -> None:
    print(f"=== Integrated PCD-to-Depth Pipeline (Closest-Line Mode) ===")
    print(f"Parent folder: {parent_folder}")
    print(f"Closest-line parameters:")
    print(f"  - Ground Z range: [{ground_z_min:.3f}, {ground_z_max:.3f}]")
    print(f"  - XY distance range: [{min_xy_distance:.3f}, {xy_radius_threshold:.3f}]")

    # Resolve input (pcd) directory and output base directory robustly
    if (parent_folder / "pcd").is_dir():
        base_dir = parent_folder
        pcd_input_dir = parent_folder / "pcd"
    elif parent_folder.name.lower() == "pcd":
        base_dir = parent_folder.parent
        pcd_input_dir = parent_folder
    elif list(parent_folder.glob("*.pcd")):
        # Parent directly holds .pcd files
        base_dir = parent_folder
        pcd_input_dir = parent_folder
    else:
        print(f"Error: Could not locate 'pcd' input. Provide either:")
        print(f"  - a folder that contains a 'pcd' subfolder, or")
        print(f"  - the 'pcd' folder itself, or")
        print(f"  - a folder that directly contains .pcd files.")
        print(f"Tried: {parent_folder}")
        return

    # Output directories under base_dir (parent of 'pcd' if parent_folder == 'pcd')
    newest_pcd_dir = base_dir / "newest_pcd"
    newest_depth_maps_dir = base_dir / "newest_depth_maps"
    newest_viz_results_dir = base_dir / "newest_viz_results"
    newest_colormap_dir = base_dir / "newest_colormap"

    newest_pcd_dir.mkdir(parents=True, exist_ok=True)
    newest_depth_maps_dir.mkdir(parents=True, exist_ok=True)
    newest_viz_results_dir.mkdir(parents=True, exist_ok=True)
    newest_colormap_dir.mkdir(parents=True, exist_ok=True)

    print(f"[DEBUG] Output directories created under: {base_dir}")
    print(f"  - newest_pcd: {newest_pcd_dir}")
    print(f"  - newest_depth_maps: {newest_depth_maps_dir}")
    print(f"  - newest_viz_results: {newest_viz_results_dir}")
    print(f"  - newest_colormap: {newest_colormap_dir}")

    if not pcd_input_dir.exists():
        print(f"Error: PCD input directory not found: {pcd_input_dir}")
        return

    # [ADD] Initialize calibration and projector before processing loop
    calib_db = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD)
    projector = LidarCameraProjector(calib_db)

    # Find PCD files
    pcd_files = sorted(list(pcd_input_dir.glob("*.pcd")))
    if not pcd_files:
        print(f"Error: No PCD files found in {pcd_input_dir}")
        return

    print(f"Processing {len(pcd_files)} PCD files...")

    processed_count = 0
    failed_count = 0

    for pcd_path in tqdm(pcd_files, desc="Processing PCDs"):
        try:
            print(f"\n[PROCESS] Starting {pcd_path.name}...")

            # Step 1: Generate closest-line (diagnostic) and load original points
            closest_line_points, original_points = find_nearest_road_point_and_generate_synthetic_pcd(
                pcd_path=pcd_path,
                ground_z_min=ground_z_min,
                ground_z_max=ground_z_max,
                min_xy_distance_from_origin=min_xy_distance,
                xy_radius_threshold=xy_radius_threshold,
                y_min=y_min,
                y_max=y_max,
                num_radius_divisions=num_radius_divisions,
                points_per_circle=points_per_circle,
                keep_original_points=keep_original_points,
                exclude_outermost_circle=exclude_outermost_circle
            )

            if original_points is None or original_points.size == 0:
                print(f"[ERROR] Empty original cloud for {pcd_path.name}")
                failed_count += 1
                continue

            # Step 1a: Build c_circles (black rings) PCD points from original cloud
            c_circles_points = build_c_circles_points_from_cloud(
                original_points,
                y_zero_band=0.01,
                blue_depth_z=BLUE_DEPTH_Z,
                num_c_radii=NUM_C_RADII,
                circle_segs=CIRCLE_SEGS,
                skip_far_count=SKIP_FAR_COUNT,
                min_first_radius=MIN_FIRST_RADIUS,
                xy_min_separation=XY_MIN_SEPARATION,
                radius_distribution=RADIUS_DISTRIBUTION,
                near_bias=NEAR_BIAS,
            )

            if c_circles_points.size == 0:
                print(f"[WARN] No c_circles points generated for {pcd_path.name}. Fallback to closest-line points.")
                if closest_line_points.size == 0:
                    failed_count += 1
                    continue
                synthetic_points = closest_line_points
            else:
                synthetic_points = c_circles_points

            # Step 1b: Merge with original PCD if requested (original + c_circles)
            if keep_original_points:
                try:
                    points_to_use = np.vstack([original_points, synthetic_points])
                except Exception as _e:
                    print(f"[WARN] Merge failed ({_e}), using synthetic only.")
                    points_to_use = synthetic_points
            else:
                points_to_use = synthetic_points

            # === NEW: 분리 투영 검증 ===
            orig_pts = original_points
            synth_only = synthetic_points
            lbl_orig = np.zeros((orig_pts.shape[0],), dtype=np.int8)
            lbl_synth = np.ones((synth_only.shape[0],), dtype=np.int8)

            depth_orig, _ = projector.project_cloud_to_depth_map_with_labels(
                camera_name, orig_pts, lbl_orig, reference_image_size
            )
            depth_synth, _ = projector.project_cloud_to_depth_map_with_labels(
                camera_name, synth_only, lbl_synth, reference_image_size
            )

            merged_pts = np.vstack([orig_pts, synth_only])
            merged_labels = np.concatenate([lbl_orig, lbl_synth])
            depth_merge, prov = projector.project_cloud_to_depth_map_with_labels(
                camera_name, merged_pts, merged_labels, reference_image_size
            )

            # 영향 픽셀 계산
            changed_mask = (depth_merge > 0) & ((depth_orig == 0) | (np.abs(depth_merge - depth_orig) > 1e-4))
            synth_new_pixels = int(np.sum((depth_orig == 0) & (depth_synth > 0)))
            synth_override_pixels = int(np.sum((depth_orig > 0) & (depth_merge != depth_orig) & (prov == 1)))
            total_valid_merge = int(np.sum(depth_merge > 0))

            print(f"[DIFF] new_pixels_from_synth={synth_new_pixels}, overrides={synth_override_pixels}, total_valid={total_valid_merge}")

            # 저장 (간단한 8bit 시각화)
            debug_dir = newest_depth_maps_dir / "debug_diff"
            debug_dir.mkdir(exist_ok=True, parents=True)
            def save_debug(name, dm):
                dm_u8 = np.clip(dm / (np.percentile(dm[dm>0],95)+1e-6) * 255, 0, 255).astype(np.uint8)
                cv2.imwrite(str(debug_dir / f"{pcd_path.stem}_{name}.png"), dm_u8)

            save_debug("orig_only", depth_orig)
            save_debug("synth_only", depth_synth)
            save_debug("merged", depth_merge)

            # provenance 컬러: -1=white, 0=blue,1=red
            prov_vis = np.full((*prov.shape,3), 255, np.uint8)
            prov_vis[prov==0] = (255,0,0)   # BGR: original=Blue channel? (파랑)
            prov_vis[prov==1] = (0,0,255)   # synthetic=Red
            cv2.imwrite(str(debug_dir / f"{pcd_path.stem}_provenance.png"), prov_vis)

            # 기존 save_depth_map 대체
            depth_map_path = newest_depth_maps_dir / f"{pcd_path.stem}.png"
            save_depth_map(depth_map_path, depth_merge)

            # Step 2: Save output PCD
            newest_pcd_path = newest_pcd_dir / f"{pcd_path.stem}.pcd"
            save_synthetic_pcd(points_to_use, newest_pcd_path)
            print(f"[SAVE] Output PCD saved: {newest_pcd_path}")

            # Step 3: Generate depth map using merged/synthetic points
            depth_map = projector.project_cloud_to_depth_map(
                camera_name, points_to_use, reference_image_size
            )

            if depth_map is None:
                print(f"[ERROR] Failed to generate depth map for {pcd_path.name}")
                failed_count += 1
                continue

            # Step 4: Save outputs
            viz_path = newest_viz_results_dir / f"{pcd_path.stem}_depth_analysis.png"
            create_depth_visualization(depth_map, viz_path, f"C-Circles Depth Map - {pcd_path.stem}")

            colormap_path = newest_colormap_dir / f"{pcd_path.stem}_colorized.png"
            create_depth_colormap_image(depth_map, colormap_path)

            processed_count += 1
            print(f"[SUCCESS] Completed {pcd_path.name}")

        except Exception as e:
            print(f"[ERROR] Processing {pcd_path.name}: {e}")
            import traceback
            traceback.print_exc()
            failed_count += 1

    print(f"\n=== Pipeline Complete ===")
    print(f"Successfully processed: {processed_count} files")
    print(f"Failed: {failed_count} files")
    print(f"Output directories:")
    print(f"  - Closest-line PCDs: {newest_pcd_dir}")
    print(f"  - Raw depth maps (16bit): {newest_depth_maps_dir}")
    print(f"  - Analysis plots: {newest_viz_results_dir}")
    print(f"  - Colorized images: {newest_colormap_dir}")

def test_single_file(pcd_file_path: str, num_divisions: int = 20, points_per_circle: int = 200, exclude_outermost: bool = True) -> None:
    """단일 파일 테스트용 함수"""
    pcd_path = Path(pcd_file_path)
    if not pcd_path.exists():
        print(f"Error: File not found: {pcd_path}")
        return
    
    # [FIX] synced_data 디렉토리를 직접 parent_folder로 사용
    # 예상 구조: ncdb-cls-sample/synced_data/pcd/xxxx.pcd
    pcd_parent = pcd_path.parent  # pcd 폴더
    synced_data_dir = pcd_parent.parent  # synced_data 폴더
    
    # 임시 테스트 디렉토리를 synced_data 아래에 생성
    test_dir = synced_data_dir / "test_single_file"
    temp_pcd_dir = test_dir / "pcd"
    temp_pcd_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy file to temporary structure
    import shutil
    temp_pcd_path = temp_pcd_dir / pcd_path.name
    if not temp_pcd_path.exists():
        shutil.copy2(pcd_path, temp_pcd_path)
    
    print(f"=== Testing single file: {pcd_path.name} ===")
    print(f"Temporary test directory: {test_dir}")
    print(f"[DEBUG] test_single_file received parameters:")
    print(f"  - num_divisions: {num_divisions}")
    print(f"  - points_per_circle: {points_per_circle}")
    print(f"  - exclude_outermost: {exclude_outermost}")
    
    # [FIX] synced_data를 parent_folder로 전달하여 바로 그 아래에 폴더 생성
    run_integrated_pipeline(
        parent_folder=synced_data_dir,  # synced_data 레벨에서 실행
        num_radius_divisions=num_divisions,
        points_per_circle=points_per_circle,
        exclude_outermost_circle=exclude_outermost
    )
    
    print(f"\n=== Test Complete ===")
    print(f"Results are in:")
    print(f"  - {synced_data_dir / 'newest_pcd'}")
    print(f"  - {synced_data_dir / 'newest_depth_maps'}")  
    print(f"  - {synced_data_dir / 'newest_viz_results'}")
    print(f"  - {synced_data_dir / 'newest_colormap'}")

def main():
    parser = argparse.ArgumentParser(description="Integrated PCD to Depth Map Pipeline")
    parser.add_argument("--parent_folder", type=str, required=True,
                        help="Parent folder containing 'pcd' directory")
    parser.add_argument("--camera", type=str, default="a6",
                        help="Camera name (default: a6)")
    parser.add_argument("--ground_z_min", type=float, default=-0.95,
                        help="Minimum ground Z coordinate")
    parser.add_argument("--ground_z_max", type=float, default=0.5,
                        help="Maximum ground Z coordinate")
    parser.add_argument("--min_xy_distance", type=float, default=1.0,
                        help="Minimum XY distance from origin")
    parser.add_argument("--xy_radius_threshold", type=float, default=10.0,
                        help="Maximum XY radius threshold")
    parser.add_argument("--y_min", type=float, default=1.5,
                        help="Minimum Y coordinate filter (ignored in closest-line mode)")
    parser.add_argument("--y_max", type=float, default=3.0,
                        help="Maximum Y coordinate filter (ignored in closest-line mode)")
    parser.add_argument("--num_divisions", type=int, default=20,      
                        help="(compat) Number of concentric circle divisions (unused)")
    parser.add_argument("--points_per_circle", type=int, default=1024, 
                        help="(compat) Points per circle (unused)")
    parser.add_argument("--test_file", type=str, default=None,
                        help="Test with single PCD file")
    parser.add_argument("--synthetic_only", action="store_true",
                        help="(compat) Save only synthetic points (unused)")
    parser.add_argument("--exclude_outermost", action="store_true", default=True,
                        help="(compat) Exclude outermost circle (unused)")
    parser.add_argument("--include_outermost", action="store_true",
                        help="(compat) Include all circles (unused)")
    
    args = parser.parse_args()

    # Maintain compatibility flags (unused now)
    if args.include_outermost:
        exclude_outermost_circle = False
    else:
        exclude_outermost_circle = args.exclude_outermost

    if args.test_file:
        test_single_file(
            pcd_file_path=args.test_file,
            num_divisions=args.num_divisions, 
            points_per_circle=args.points_per_circle, 
            exclude_outermost=exclude_outermost_circle
        )
    else:
        parent_folder = Path(args.parent_folder)
        if not parent_folder.exists():
            print(f"Error: Parent folder not found: {parent_folder}")
            return

        run_integrated_pipeline(
            parent_folder=parent_folder,
            camera_name=args.camera,
            ground_z_min=args.ground_z_min,
            ground_z_max=args.ground_z_max,
            min_xy_distance=args.min_xy_distance,
            xy_radius_threshold=args.xy_radius_threshold,
            y_min=args.y_min,
            y_max=args.y_max,
            num_radius_divisions=args.num_divisions,   # compat
            points_per_circle=args.points_per_circle,   # compat
            keep_original_points=not args.synthetic_only,  # compat
            exclude_outermost_circle=exclude_outermost_circle  # compat
        )

if __name__ == "__main__":
    main()

# integrated_pcd_depth_pipeline.py