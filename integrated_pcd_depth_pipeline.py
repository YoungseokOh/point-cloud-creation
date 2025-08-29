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
    exclude_outermost_circle: bool = True  # [NEW] 가장 큰 원 제외 옵션
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    PCD에서 가장 가까운 도로점을 찾고, 동심원 합성 포인트 클라우드를 생성합니다.
    Returns: (combined_points, closest_point_info)
    - combined_points: 기존 포인트 + 합성 동심원 포인트 (keep_original_points=True인 경우)
    """
    try:
        # [DEBUG] 함수 시작 시 파라미터 출력
        print(f"[DEBUG] find_nearest_road_point_and_generate_synthetic_pcd called with:")
        print(f"  - num_radius_divisions: {num_radius_divisions}")
        print(f"  - points_per_circle: {points_per_circle}")
        print(f"  - exclude_outermost_circle: {exclude_outermost_circle}")
        
        # Step 1: PCD 파일 로드
        if OPEN3D_AVAILABLE:
            pcd = o3d.io.read_point_cloud(str(pcd_path))
            original_points = np.asarray(pcd.points)
        else:
            # Fallback ASCII parser
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
            original_points = np.array(points)
        
        if original_points.size == 0:
            print(f"Warning: No points found in {pcd_path}")
            return np.empty((0, 3)), None

        print(f"[DEBUG] Loaded {len(original_points):,} original points from {pcd_path.name}")

        # Step 2: 도로점 찾기를 위한 필터링 (원본은 건드리지 않음)
        # Z좌표 기반 지면 후보 필터링
        initial_ground_indices = np.where(
            (original_points[:, 2] > ground_z_min) & (original_points[:, 2] < ground_z_max)
        )[0]
        
        if len(initial_ground_indices) == 0:
            print(f"Warning: No ground candidates found in {pcd_path}")
            if keep_original_points:
                print(f"[DEBUG] Returning original points only: {len(original_points):,}")
                return original_points, None
            else:
                return np.empty((0, 3)), None
        
        initial_ground_points = original_points[initial_ground_indices]
        print(f"[DEBUG] Found {len(initial_ground_points):,} ground candidate points")
        
        # XY 거리 계산
        xy_distances_sq = initial_ground_points[:, 0]**2 + initial_ground_points[:, 1]**2
        
        # 거리 및 Y축 범위 필터링
        min_radius = min_xy_distance_from_origin
        filter_conditions = (
            (xy_distances_sq > min_radius**2) & 
            (xy_distances_sq < xy_radius_threshold**2)
        )
        
        if y_min is not None:
            filter_conditions &= (initial_ground_points[:, 1] > y_min)
        if y_max is not None:
            filter_conditions &= (initial_ground_points[:, 1] < y_max)
        
        filtered_indices = np.where(filter_conditions)[0]
        
        if len(filtered_indices) == 0:
            print(f"Warning: No road candidates found in {pcd_path}")
            if keep_original_points:
                print(f"[DEBUG] Returning original points only: {len(original_points):,}")
                return original_points, None
            else:
                return np.empty((0, 3)), None
        
        final_road_points = initial_ground_points[filtered_indices]
        distances_sq_to_origin = final_road_points[:, 0]**2 + final_road_points[:, 1]**2
        
        # 가장 가까운 도로점 선택
        closest_index = np.argmin(distances_sq_to_origin)
        closest_point = final_road_points[closest_index]
        
        print(f"[DEBUG] Found closest road point at: ({closest_point[0]:.3f}, {closest_point[1]:.3f}, {closest_point[2]:.3f})")

        # Step 3: 동심원 합성 포인트 생성
        new_radius_center = np.array([0.0, 0.0, closest_point[2]])
        all_circle_points = []
        
        # [FIX] 가장 큰 원 제외 로직
        if exclude_outermost_circle:
            # 가장 큰 원을 제외하고 생성 (1부터 num_radius_divisions-1까지)
            effective_divisions = num_radius_divisions - 1
            range_end = num_radius_divisions  # 1부터 19까지 (20번째 제외)
            print(f"[DEBUG] Excluding outermost circle. Generating {effective_divisions} circles (1 to {range_end-1})")
        else:
            # 기존 방식: 모든 원 생성 (1부터 num_radius_divisions까지)
            effective_divisions = num_radius_divisions
            range_end = num_radius_divisions + 1  # 1부터 20까지
            print(f"[DEBUG] Including all circles. Generating {effective_divisions} circles (1 to {num_radius_divisions})")
        
        for i in range(1, range_end):
            # [FIX] 반지름 계산 방식 수정
            if exclude_outermost_circle:
                # 가장 큰 원 제외: 1~19를 1~20 스케일로 매핑
                # i=1: (1/20) * min_radius, i=19: (19/20) * min_radius
                current_radius = (i / num_radius_divisions) * min_radius
            else:
                # 기존 방식: i=1: (1/20), i=20: (20/20) = 1.0
                current_radius = (i / num_radius_divisions) * min_radius
            
            theta = np.linspace(0, 2 * np.pi, points_per_circle)
            circle_x = current_radius * np.cos(theta)
            circle_y = current_radius * np.sin(theta)
            circle_z = np.full(points_per_circle, new_radius_center[2])
            circle_points_3d = np.column_stack((circle_x, circle_y, circle_z))
            all_circle_points.append(circle_points_3d)
            
            if i <= 3:  # 처음 3개 원만 디버그 출력
                print(f"[DEBUG] Circle {i}: radius={current_radius:.3f}m, points={len(circle_points_3d)}")
        
        synthetic_points = np.vstack(all_circle_points)
        print(f"[DEBUG] Generated {len(synthetic_points):,} synthetic concentric circle points")
        
        # [DEBUG] 실제로 생성된 포인트 수 검증
        expected_count = effective_divisions * points_per_circle
        if len(synthetic_points) != expected_count:
            print(f"[WARNING] Expected {expected_count:,} synthetic points, but got {len(synthetic_points):,}")
        else:
            print(f"[VERIFY] ✅ Synthetic point count matches expectation: {len(synthetic_points):,}")
        
        # Step 4: 기존 포인트와 합성 포인트 결합
        if keep_original_points:
            combined_points = np.vstack([original_points, synthetic_points])
            print(f"[DEBUG] Combined result: {len(original_points):,} original + {len(synthetic_points):,} synthetic = {len(combined_points):,} total points")
        else:
            combined_points = synthetic_points
            print(f"[DEBUG] Synthetic only: {len(synthetic_points):,} points")
        
        return combined_points, closest_point
        
    except Exception as e:
        print(f"Error processing {pcd_path}: {e}")
        return np.empty((0, 3)), None

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
        ax1.set_title(f'{title}\n({len(valid_depths):,} valid pixels)')
        ax1.axis('off')
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
        colored_depth[mask] = [0, 0, 0]
        
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
    num_radius_divisions: int = 20,    # [FIX] 기본값을 20으로 설정
    points_per_circle: int = 200,      # [FIX] 기본값을 200으로 설정
    reference_image_size: Tuple[int, int] = (1920, 1536),
    keep_original_points: bool = True,
    exclude_outermost_circle: bool = True  # [NEW] 추가 파라미터
) -> None:
    """
    통합 파이프라인: PCD → 합성 PCD → Depth Map → 시각화
    """
    print(f"=== Integrated PCD-to-Depth Pipeline ===")
    print(f"Parent folder: {parent_folder}")
    print(f"Keep original points: {keep_original_points}")
    
    # [DEBUG] 실제로 받은 파라미터 출력
    print(f"Synthetic circle parameters (received in run_integrated_pipeline):")
    print(f"  - Radius divisions: {num_radius_divisions}")
    print(f"  - Points per circle: {points_per_circle}")
    print(f"  - Expected synthetic points: {num_radius_divisions * points_per_circle:,}")
    print(f"  - Ground Z range: [{ground_z_min:.3f}, {ground_z_max:.3f}]")
    print(f"  - XY distance range: [{min_xy_distance:.3f}, {xy_radius_threshold:.3f}]")
    if y_min is not None or y_max is not None:
        print(f"  - Y filter: [{y_min}, {y_max}]")
    
    # [FIX] 출력 디렉토리를 parent_folder 바로 아래에 생성
    # 기존: parent_folder.parent / f"synthetic_depth_output_{parent_folder.name}"
    # 수정: parent_folder 바로 아래에 각 폴더 생성
    new_pcd_dir = parent_folder / "new_pcd"
    new_depth_maps_dir = parent_folder / "new_depth_maps"
    new_viz_results_dir = parent_folder / "new_viz_results"
    new_colormap_dir = parent_folder / "new_colormap"
    
    # 디렉토리 생성
    new_pcd_dir.mkdir(parents=True, exist_ok=True)
    new_depth_maps_dir.mkdir(parents=True, exist_ok=True)
    new_viz_results_dir.mkdir(parents=True, exist_ok=True)
    new_colormap_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"[DEBUG] Output directories created under: {parent_folder}")
    print(f"  - new_pcd: {new_pcd_dir}")
    print(f"  - new_depth_maps: {new_depth_maps_dir}")
    print(f"  - new_viz_results: {new_viz_results_dir}")
    print(f"  - new_colormap: {new_colormap_dir}")
    
    # Setup input directories
    pcd_input_dir = parent_folder / "pcd"
    if not pcd_input_dir.exists():
        print(f"Error: PCD input directory not found: {pcd_input_dir}")
        return
    
    # Initialize calibration and projector
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
            print(f"[DEBUG] Calling find_nearest_road_point_and_generate_synthetic_pcd with:")
            print(f"  - num_radius_divisions: {num_radius_divisions}")
            print(f"  - points_per_circle: {points_per_circle}")
            print(f"  - exclude_outermost_circle: {exclude_outermost_circle}")
            
            # Step 1: Generate synthetic PCD with outermost circle exclusion option
            combined_points, closest_point = find_nearest_road_point_and_generate_synthetic_pcd(
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
            
            if combined_points.size == 0:
                print(f"[ERROR] No points generated for {pcd_path.name}")
                failed_count += 1
                continue
            
            # Step 2: Save combined PCD (original + synthetic)
            new_pcd_path = new_pcd_dir / f"{pcd_path.stem}.pcd"
            save_synthetic_pcd(combined_points, new_pcd_path)
            print(f"[SAVE] Combined PCD saved: {new_pcd_path}")
            
            # Step 3: Generate depth map (using combined points)
            depth_map = projector.project_cloud_to_depth_map(
                camera_name, combined_points, reference_image_size
            )
            
            if depth_map is None:
                print(f"[ERROR] Failed to generate depth map for {pcd_path.name}")
                failed_count += 1
                continue
            
            # Step 4: Save outputs
            depth_map_path = new_depth_maps_dir / f"{pcd_path.stem}.png"
            save_depth_map(depth_map_path, depth_map)
            print(f"[SAVE] Depth map saved: {depth_map_path}")
            
            viz_path = new_viz_results_dir / f"{pcd_path.stem}_depth_analysis.png"
            create_depth_visualization(depth_map, viz_path, f"Synthetic Depth Map - {pcd_path.stem}")
            
            colormap_path = new_colormap_dir / f"{pcd_path.stem}_colorized.png"
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
    print(f"  - Synthetic PCDs: {new_pcd_dir}")
    print(f"  - Raw depth maps (16bit): {new_depth_maps_dir}")
    print(f"  - Analysis plots: {new_viz_results_dir}")
    print(f"  - Colorized images: {new_colormap_dir}")

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
        parent_folder=synced_data_dir,  # [FIX] synced_data 레벨에서 실행
        num_radius_divisions=num_divisions,
        points_per_circle=points_per_circle,
        exclude_outermost_circle=exclude_outermost
    )
    
    print(f"\n=== Test Complete ===")
    # [FIX] 출력 경로도 수정
    print(f"Results are in:")
    print(f"  - {synced_data_dir / 'new_pcd'}")
    print(f"  - {synced_data_dir / 'new_depth_maps'}")  
    print(f"  - {synced_data_dir / 'new_viz_results'}")
    print(f"  - {synced_data_dir / 'new_colormap'}")

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
                        help="Minimum Y coordinate filter")
    parser.add_argument("--y_max", type=float, default=3.0,
                        help="Maximum Y coordinate filter")
    parser.add_argument("--num_divisions", type=int, default=20,      
                        help="Number of concentric circle divisions (default: 20)")
    parser.add_argument("--points_per_circle", type=int, default=1024, 
                        help="Points per circle (default: 200)")
    parser.add_argument("--test_file", type=str, default=None,
                        help="Test with single PCD file")
    parser.add_argument("--synthetic_only", action="store_true",
                        help="Save only synthetic concentric circles (exclude original PCD points)")
    # [FIX] exclude_outermost argument 추가
    parser.add_argument("--exclude_outermost", action="store_true", default=True,
                        help="Exclude the outermost (largest) concentric circle (default: True)")
    parser.add_argument("--include_outermost", action="store_true",
                        help="Include all concentric circles (override --exclude_outermost)")
    
    args = parser.parse_args()
    
    # [FIX] exclude_outermost 로직 수정
    if args.include_outermost:
        exclude_outermost_circle = False
    else:
        exclude_outermost_circle = args.exclude_outermost
    
    if args.test_file:
        # [FIX] 모든 파라미터 전달
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
        
        # [FIX] 명시적으로 모든 파라미터 전달
        run_integrated_pipeline(
            parent_folder=parent_folder,
            camera_name=args.camera,
            ground_z_min=args.ground_z_min,
            ground_z_max=args.ground_z_max,
            min_xy_distance=args.min_xy_distance,
            xy_radius_threshold=args.xy_radius_threshold,
            y_min=args.y_min,
            y_max=args.y_max,
            num_radius_divisions=args.num_divisions,
            points_per_circle=args.points_per_circle,
            keep_original_points=not args.synthetic_only,
            exclude_outermost_circle=exclude_outermost_circle
        )

if __name__ == "__main__":
    main()

# integrated_pcd_depth_pipeline.py