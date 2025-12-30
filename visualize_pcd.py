"""
PCD를 이미지에 투영하는 고급 시각화 도구

기능:
1. Open3D를 이용한 3D 포인트 클라우드 시각화
2. PCD를 카메라 이미지에 투영 (깊이값 색상 표시)
3. VADAS Fisheye 카메라 모델 사용
4. 투영된 포인트를 RGB 이미지에 오버레이
"""

import os
import sys
import json
import struct
import math
import argparse
import copy
import numpy as np
import cv2
from pathlib import Path
from typing import Tuple, Optional
from PIL import Image, ImageDraw

from ref.ref_calibration_data import (
    DEFAULT_CALIB as REF_DEFAULT_CALIB,
    DEFAULT_LIDAR_TO_WORLD_v3,
)

# =============================================================================
# 1. 캘리브레이션 데이터 (ref_calibration_data.py 참고)
# =============================================================================

# ref_calibration_data.py 내용 그대로 활용
DEFAULT_CALIB = copy.deepcopy(REF_DEFAULT_CALIB)

# 최신 LiDAR→World 행렬은 ref 데이터에서 가져옴
DEFAULT_LIDAR_TO_WORLD = DEFAULT_LIDAR_TO_WORLD_v3.copy()

# 추가 회전 옵션 (LiDAR 좌표계를 Z축 기준으로 180도 회전)
ENABLE_LIDAR_YAW_180 = True
ROTATE_LIDAR_180_Z = np.array([
    [-1.,  0.,  0., 0.],
    [ 0., -1.,  0., 0.],
    [ 0.,  0.,  1., 0.],
    [ 0.,  0.,  0., 1.]
])

# Z축 180도 회전 (Z값 반전)
ROTATE_Z_180 = np.array([
    [ 1.,  0.,  0., 0.],
    [ 0.,  1.,  0., 0.],
    [ 0.,  0., -1., 0.],
    [ 0.,  0.,  0., 1.]
])

# LiDAR PCD가 모두 양수값이라는 데이터 특성 반영 옵션 (기본: False)
FORCE_POSITIVE_LIDAR = True

# 전방 스트립 제외 옵션 (ref_camera_lidar_projector와 동일 조건)
FILTER_FRONT_STRIP = True
FRONT_STRIP_Y_RANGE = (-0.7, 0.5)
FRONT_STRIP_X_MIN = 0.0

# =============================================================================
# 2. VADAS Fisheye 카메라 모델
# =============================================================================

class VADASFisheyeCameraModel:
    """VADAS Polynomial Fisheye Camera Model
    
    기준 논문/구현:
    - ref_camera_lidar_projector.py (integrated_pcd_depth_pipeline_newest.py에서 사용)
    - 테스트: test_640x384_div_comparison.py (검증됨)
    
    Intrinsic 파라미터 (11개):
        [0:7]  : k[0~6] - Polynomial 계수 (fisheyeParam.u2d)
        [7]    : s - Focal length scale factor
        [8]    : div - Normalization divisor (CRITICAL)
        [9]    : ux - Principal point X offset
        [10]   : uy - Principal point Y offset
    """
    
    def __init__(self, intrinsic: list, image_size: Optional[Tuple[int, int]] = None):
        """
        Args:
            intrinsic: [k0, k1, k2, k3, k4, k5, k6, s, div, ux, uy]
            image_size: (width, height)
        """
        if len(intrinsic) < 11:
            raise ValueError("VADAS intrinsic must have at least 11 parameters")
        
        self.intrinsic = intrinsic
        self.image_size = image_size
        self.original_intrinsic = list(intrinsic)
        
        # Polynomial 계수 (k0~k6: 7개)
        self.k = intrinsic[0:7]
        # VADAS 특수 파라미터
        self.s = intrinsic[7]      # Focal length scale
        self.div = intrinsic[8]    # Normalization divisor
        self.ux = intrinsic[9]     # Principal point X offset
        self.uy = intrinsic[10]    # Principal point Y offset
        
        # Aspect ratio scaling (다른 해상도용)
        self.scale_x = 1.0
        self.scale_y = 1.0
    
    def _poly_eval(self, coeffs: list, x: float) -> float:
        """Polynomial 평가 (Horner's method)"""
        res = 0.0
        for c in reversed(coeffs):
            res = res * x + c
        return res
    
    def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
        """
        카메라 좌표 (Xc, Yc, Zc)를 이미지 좌표 (u, v)로 투영
        
        VADAS 모델 (기본 +X 전방 축):
        - 카메라가 +X 축 방향을 바라봄 (정면)
        - Y는 오른쪽, Z는 아래쪽
        - 극좌표: nx = -Yc, ny = -Zc (카메라 C++ 구현과 동일)
        - Theta = atan2(dist(nx, ny), Xc)
        
        투영 과정:
        1. 극좌표 변환 (Y/Z 평면)
        2. Theta 계산
        3. Polynomial 평가: xd = theta * s
        4. Normalization: rd = poly(xd) / div
        5. 이미지 좌표: u, v (aspect ratio 포함)
        
        Args:
            Xc, Yc, Zc: 카메라 좌표 (3D point in camera frame)
        
        Returns:
            (u, v, valid): 이미지 좌표 및 유효성 플래그
        """
        # 극좌표 변환 (카메라 +X 전방 기준)
        #   forward_axis (빨간색, +X) -> Xc
        #   horizontal_axis (초록색, +Y) -> -Yc
        #   vertical_axis (파란색, +Z) -> -Zc
        forward = Zc
        horiz = -Xc
        vert = -Yc
        
        dist = math.hypot(horiz, vert)
        
        # 극점 처리 (거의 정면일 때)
        if dist < 1e-10:
            dist = 1e-10
        
        # 각도 성분 (수평/수직은 LiDAR 시각화에서 초록/파랑 방향)
        cosPhi = horiz / dist
        sinPhi = vert / dist
        
        # 각도 Theta 계산 (빨간색 +X 축 사용)
        theta = math.atan2(dist, forward)
        
        # 전방축이 음수면 투영 불가능
        if forward < 0:
            return 0, 0, False
        
        # Polynomial 입력
        xd = theta * self.s
        
        # Normalization divisor 체크
        if abs(self.div) < 1e-9:
            return 0, 0, False
        
        # Polynomial 평가 및 정규화
        rd = self._poly_eval(self.k, xd) / self.div
        
        # NaN/Inf 체크
        if math.isinf(rd) or math.isnan(rd):
            return 0, 0, False
        
        # 이미지 중심 오프셋
        img_w_half = (self.image_size[0] / 2) if self.image_size else 0
        img_h_half = (self.image_size[1] / 2) if self.image_size else 0
        
        # 이미지 좌표 계산 (aspect ratio scaling 포함)
        u = rd * cosPhi * self.scale_x + self.ux + img_w_half
        v = rd * sinPhi * self.scale_y + self.uy + img_h_half
        
        # 결과 반환
        return int(round(u)), int(round(v)), True
    
    def set_image_size(self, image_size: Tuple[int, int]):
        """이미지 크기 설정"""
        self.image_size = image_size


# =============================================================================
# 3. 보정 데이터 관리
# =============================================================================

class CalibrationDB:
    """카메라 보정 데이터 관리"""
    
    def __init__(self, calib_dict: dict, lidar_to_world: Optional[np.ndarray] = None):
        """
        Args:
            calib_dict: 보정 데이터 딕셔너리
            lidar_to_world: LiDAR to World 변환 행렬 (4x4)
        """
        self.cameras = {}
        self.lidar_to_world = lidar_to_world if lidar_to_world is not None else np.eye(4)
        
        for cam_name, calib_data in calib_dict.items():
            intrinsic = calib_data["intrinsic"]
            extrinsic_raw = calib_data["extrinsic"]
            image_size = tuple(calib_data["image_size"]) if calib_data.get("image_size") else None
            
            # Rodrigues 벡터 → 회전 행렬 (tx, ty, tz, rx, ry, rz 순서)
            extrinsic_matrix = self._rodrigues_to_matrix(extrinsic_raw)
            lidar_to_cam_matrix = extrinsic_matrix @ self.lidar_to_world

            # Z축 180도 회전 적용
            lidar_to_cam_matrix = lidar_to_cam_matrix
            
            camera_model = VADASFisheyeCameraModel(intrinsic, image_size=image_size)
            
            self.cameras[cam_name] = {
                "model": camera_model,
                "extrinsic": extrinsic_matrix,
                "intrinsic": intrinsic,
                "lidar_to_camera": lidar_to_cam_matrix
            }
    
    def _rodrigues_to_matrix(self, rodrigues_vec: list) -> np.ndarray:
        """Rodrigues 벡터를 4x4 변환 행렬로 변환"""
        if len(rodrigues_vec) == 6:
            # [tx, ty, tz, rx, ry, rz]
            tvec = np.array(rodrigues_vec[:3], dtype=np.float32)
            rvec = np.array(rodrigues_vec[3:6], dtype=np.float32)
            
            # OpenCV의 Rodrigues 변환
            R, _ = cv2.Rodrigues(rvec)
            
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec
            return T
        else:
            return np.array(rodrigues_vec).reshape(4, 4)
    
    def get(self, camera_name: str) -> dict:
        """카메라 정보 조회"""
        return self.cameras[camera_name]


# =============================================================================
# 4. PCD 파일 읽기
# =============================================================================

def load_pcd_xyz(pcd_path: Path) -> np.ndarray:
    """PCD 파일에서 XYZ 좌표만 추출 (Binary/ASCII 모두 지원)"""
    with open(pcd_path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header를 읽는 중 파일이 끝났습니다")
            decoded = line.decode('utf-8', errors='ignore').strip()
            header_lines.append(decoded)
            if decoded.startswith('DATA'):
                break

        num_points = 0
        data_format = 'ascii'
        fields = []
        sizes = []
        types = []
        counts = []

        for line in header_lines:
            tokens = line.split()
            if not tokens:
                continue
            key = tokens[0].upper()
            if key == 'POINTS':
                num_points = int(tokens[1])
            elif key == 'DATA':
                data_format = tokens[1]
            elif key == 'FIELDS':
                fields = tokens[1:]
            elif key == 'SIZE':
                sizes = list(map(int, tokens[1:]))
            elif key == 'TYPE':
                types = tokens[1:]
            elif key == 'COUNT':
                counts = list(map(int, tokens[1:]))

        if not counts and fields:
            counts = [1] * len(fields)

        print(f"  포인트 수: {num_points}")
        print(f"  데이터 포맷: {data_format}")

        if data_format == 'binary':
            dtype_fields = []
            for field_name, size, typ, cnt in zip(fields, sizes, types, counts):
                if typ == 'F':
                    dtype = np.float32 if size == 4 else np.float64
                elif typ == 'U':
                    dtype = np.uint8 if size == 1 else (np.uint16 if size == 2 else np.uint32)
                elif typ == 'I':
                    dtype = np.int8 if size == 1 else (np.int16 if size == 2 else np.int32)
                else:
                    raise ValueError(f"지원되지 않는 TYPE: {typ}")

                if cnt == 1:
                    dtype_fields.append((field_name, dtype))
                else:
                    dtype_fields.append((field_name, (dtype, cnt)))

            dtype = np.dtype(dtype_fields)
            data = np.fromfile(f, dtype=dtype, count=num_points)
            if {'x', 'y', 'z'} - set(dtype.names or []):
                raise ValueError("PCD 데이터에 x/y/z 필드가 없습니다")
            xyz = np.stack([data['x'], data['y'], data['z']], axis=-1).astype(np.float32)
            return xyz

        elif data_format == 'binary_compressed':
            raise NotImplementedError("binary_compressed PCD는 아직 지원되지 않습니다")

        # ASCII fallback
        points_list = []
        for line in f:
            parts = line.decode('utf-8', errors='ignore').strip().split()
            if len(parts) >= 3:
                points_list.append([float(parts[0]), float(parts[1]), float(parts[2])])
        return np.array(points_list, dtype=np.float32)


def ensure_positive_lidar(points: np.ndarray) -> np.ndarray:
    """데이터셋 특성에 맞춰 LiDAR 좌표를 양수 범위로 보정"""
    if not FORCE_POSITIVE_LIDAR or points.size == 0:
        return points
    return np.abs(points)


def filter_front_strip(points: np.ndarray, log_prefix: str = "") -> np.ndarray:
    """ref_camera_lidar_projector와 동일하게 전방 스트립(X>=0 & -0.7<=Y<=0.5) 제거"""
    if not FILTER_FRONT_STRIP or points.size == 0:
        return points
    y_min, y_max = FRONT_STRIP_Y_RANGE
    mask_keep = ~(((points[:, 1] >= y_min) & (points[:, 1] <= y_max)) & (points[:, 0] >= FRONT_STRIP_X_MIN))
    removed = int(points.shape[0] - mask_keep.sum())
    if removed > 0 and log_prefix is not None:
        print(f"{log_prefix}↳ 전방 스트립 제외: {removed}개 (잔여 {mask_keep.sum()})")
    return points[mask_keep]


# =============================================================================
# 5. 거리에 따른 색상 계산
# =============================================================================

def get_color_from_distance(distance: float, max_distance: float = 50.0) -> Tuple[int, int, int]:
    """
    거리에 따라 BGR 색상 계산 (Jet colormap)
    
    Args:
        distance: 깊이 값 (미터)
        max_distance: 최대 거리 (미터)
    
    Returns:
        (B, G, R) 튜플
    """
    # 정규화 (0~1)
    normalized = min(distance / max_distance, 1.0)
    
    # Jet colormap (간단한 구현)
    if normalized < 0.25:
        r = 0
        g = int(255 * (normalized / 0.25))
        b = 255
    elif normalized < 0.5:
        r = 0
        g = 255
        b = int(255 * (1 - (normalized - 0.25) / 0.25))
    elif normalized < 0.75:
        r = int(255 * ((normalized - 0.5) / 0.25))
        g = 255
        b = 0
    else:
        r = 255
        g = int(255 * (1 - (normalized - 0.75) / 0.25))
        b = 0
    
    return (b, g, r)  # BGR


# =============================================================================
# 6. 포인트 클라우드를 이미지에 투영
# =============================================================================

def project_cloud_to_image(
    cloud_xyz: np.ndarray,
    image: np.ndarray,
    calib_db: CalibrationDB,
    camera_name: str = "a6",
    max_distance: float = 50.0,
    point_radius: int = 2
) -> Tuple[np.ndarray, int, int]:
    """
    포인트 클라우드를 이미지에 투영
    
    Args:
        cloud_xyz: (N, 3) 포인트 배열
        image: 입력 이미지 (BGR, uint8)
        calib_db: 보정 데이터베이스
        camera_name: 카메라 이름
        max_distance: 최대 표시 거리
        point_radius: 포인트 원의 반경 (픽셀)
    
    Returns:
        (output_image, in_front_count, on_image_count)
    """
    h, w = image.shape[:2]
    output_image = image.copy()
    
    camera_info = calib_db.get(camera_name)
    camera_model = camera_info["model"]
    extrinsic = camera_info["extrinsic"]
    lidar_to_camera = camera_info.get("lidar_to_camera")
    if lidar_to_camera is None:
        lidar_to_camera = extrinsic @ calib_db.lidar_to_world
    
    # 이미지 크기 설정
    camera_model.set_image_size((w, h))
    
    # 전처리 필터
    filtered_cloud = filter_front_strip(cloud_xyz, log_prefix="  ")

    # 좌표 변환: LiDAR → Camera
    cloud_xyz_hom = np.hstack([filtered_cloud, np.ones((filtered_cloud.shape[0], 1))])
    cam_pts_hom = (lidar_to_camera @ cloud_xyz_hom.T).T
    cam_pts = cam_pts_hom[:, :3]
    
    in_front_count = 0
    on_image_count = 0
    
    print(f"  투영 중 ({filtered_cloud.shape[0]} 포인트)...")
    
    # 각 포인트를 투영
    for idx, (Xc, Yc, Zc) in enumerate(cam_pts):
        # 카메라 앞쪽만 처리 (Xc > 0)
        if Xc <= 0:
            continue
        
        in_front_count += 1
        
        # 투영
        u, v, valid = camera_model.project_point(Xc, Yc, Zc)
        
        # 이미지 범위 확인
        if valid and 0 <= u < w and 0 <= v < h:
            on_image_count += 1
            
            # 항상 빨간색으로 표시 (BGR)
            cv2.circle(output_image, (u, v), point_radius, (0, 0, 255), -1)
    
    print(f"  카메라 앞: {in_front_count}개")
    print(f"  이미지 범위: {on_image_count}개")
    
    return output_image, in_front_count, on_image_count


# =============================================================================
# 7. Open3D 시각화
# =============================================================================

def visualize_pcd_3d_with_projection(
    pcd_path: Path,
    image_path: Path,
    calib_db: CalibrationDB,
    camera_name: str = "a6",
    max_distance: float = 50.0
):
    """
    Open3D 3D 시각화 + 투영된 포인트 강조 표시
    
    포인트 색상 의미:
        🔴 빨간색: 이미지에 성공적으로 투영된 포인트
        ⚪ 기본 색상: 투영되지 않은 포인트 (PCD 원본 색 또는 회색)
    
    Args:
        pcd_path: PCD 파일 경로
        image_path: 이미지 파일 경로
        calib_db: 보정 데이터베이스
        camera_name: 카메라 이름
        max_distance: 최대 표시 거리
    """
    try:
        import open3d as o3d
    except ImportError:
        print("❌ open3d를 설치해야 합니다: pip install open3d")
        return
    
    if not pcd_path.exists():
        print(f"❌ PCD 파일을 찾을 수 없습니다: {pcd_path}")
        return
    
    if not image_path.exists():
        print(f"⚠️  이미지 파일을 찾을 수 없습니다: {image_path}")
        image = None
    else:
        image = cv2.imread(str(image_path))
    
    try:
        print(f"\n[7] 3D 시각화 + 투영 정보")
        print("=" * 70)
        
        # PCD 로드
        cloud_xyz = load_pcd_xyz(pcd_path)
        cloud_xyz = ensure_positive_lidar(cloud_xyz)
        cloud_xyz = filter_front_strip(cloud_xyz)
        pcd = o3d.io.read_point_cloud(str(pcd_path))
        
        # 필터링된 포인트에 맞게 PCD 포인트도 필터링
        pcd_points = np.asarray(pcd.points)
        if len(pcd_points) == len(cloud_xyz):
            pass
        else:
            # 크기 불일치 시 XYZ 좌표 기준으로 새 PCD 생성
            pcd.clear()
            pcd.points = o3d.utility.Vector3dVector(cloud_xyz)
        
        print(f"  포인트: {len(cloud_xyz):,}개")
        
        # 투영 정보 계산
        camera_info = calib_db.get(camera_name)
        camera_model = camera_info["model"]
        extrinsic = camera_info["extrinsic"]
        lidar_to_camera = camera_info.get("lidar_to_camera")
        if lidar_to_camera is None:
            lidar_to_camera = extrinsic @ calib_db.lidar_to_world
        
        if image is not None:
            h, w = image.shape[:2]
            camera_model.set_image_size((w, h))
        
        # 좌표 변환: LiDAR → Camera
        cloud_xyz_hom = np.hstack([cloud_xyz, np.ones((cloud_xyz.shape[0], 1))])
        cam_pts_hom = (lidar_to_camera @ cloud_xyz_hom.T).T
        cam_pts = cam_pts_hom[:, :3]
        
        # 투영 성공 여부 표시
        projection_mask = np.zeros(len(cloud_xyz), dtype=bool)
        in_front_count = 0
        on_image_count = 0
        
        for idx, (Xc, Yc, Zc) in enumerate(cam_pts):
            if Xc <= 0:
                continue
            
            in_front_count += 1
            
            if image is not None:
                u, v, valid = camera_model.project_point(Xc, Yc, Zc)
                if valid and 0 <= u < w and 0 <= v < h:
                    projection_mask[idx] = True
                    on_image_count += 1
            else:
                projection_mask[idx] = True
                on_image_count += 1
        
        print(f"  카메라 앞: {in_front_count:,}개 ({100*in_front_count/len(cloud_xyz):.1f}%)")
        print(f"  이미지 범위: {on_image_count:,}개 ({100*on_image_count/len(cloud_xyz):.1f}%)")
        
        # 포인트 클라우드 색상화 (기본 회색 + 투영 성공 포인트만 빨간색)
        # PCD 원본 색상은 무시하고 새로운 색상만 적용
        colors = np.zeros((len(cloud_xyz), 3), dtype=np.float64)
        colors[:] = [0.4, 0.4, 0.4]  # 모든 포인트를 회색으로 초기화
        colors[projection_mask] = [1.0, 0.0, 0.0]  # 투영된 포인트만 빨간색

        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 좌표축 추가 (LiDAR 고정) + 카메라 좌표축 (빨간색 = 투영 방향)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=5.0, origin=[0, 0, 0]
        )

        camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=4.0)
        try:
            camera_to_lidar = np.linalg.inv(lidar_to_camera)
            camera_frame.transform(camera_to_lidar)
        except np.linalg.LinAlgError:
            camera_frame = None
        
        # 범례 출력
        print("\n  범례 (포인트 색상):")
        print("    🔴 빨간색 (Red): 이미지에 투영된 포인트")
        print("    ⚪ 기본 색상: 투영되지 않은 포인트 (PCD 원본 색 또는 중성 회색)")
        print("    📐 카메라 좌표축(빨간색 +X): 투영에 사용되는 기준 축")
        
        # 시각화
        print("\n  Open3D 윈도우를 열고 있습니다...")
        print("  마우스로 3D 모델을 회전할 수 있습니다.")
        geometries = [pcd, coord_frame]
        if camera_frame is not None:
            geometries.append(camera_frame)

        o3d.visualization.draw_geometries(
            geometries,
            window_name=f"PCD Projection Visualization: {pcd_path.name}",
            width=1200,
            height=800
        )
    
    except Exception as e:
        print(f"❌ 3D 시각화 오류: {e}")
        import traceback
        traceback.print_exc()


def visualize_pcd_3d(pcd_path: Path):
    """
    Open3D를 이용한 3D 포인트 클라우드 시각화
    """
    try:
        import open3d as o3d
    except ImportError:
        print("Error: open3d가 설치되지 않았습니다.")
        print("  pip install open3d 로 설치하세요.")
        return
    
    if not pcd_path.exists():
        print(f"Error: 파일을 찾을 수 없습니다: {pcd_path}")
        return
    
    try:
        print(f"\n3D 시각화 중: {pcd_path.name}")
        pcd = o3d.io.read_point_cloud(str(pcd_path))
        
        if not pcd.has_points():
            print("Error: 포인트 클라우드가 비어있습니다.")
            return
        
        # 경계 상자
        aabb = pcd.get_axis_aligned_bounding_box()
        min_bound = aabb.get_min_bound()
        max_bound = aabb.get_max_bound()
        
        print(f"  포인트 수: {len(pcd.points)}")
        print(f"  최소값: {min_bound}")
        print(f"  최대값: {max_bound}")
        
        # 시각화
        o3d.visualization.draw_geometries([pcd], window_name=f"PCD: {pcd_path.name}")
        
    except Exception as e:
        print(f"Error: {e}")


# =============================================================================
# 8. 메인 함수
# =============================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Project LiDAR PCD onto fisheye camera images"
    )
    parser.add_argument(
        "--data-root",
        type=str,
        default=str(Path("ncdb-cls-sample") / "synced_data"),
        help="Root directory containing synced data folders (default: ncdb-cls-sample/synced_data)",
    )
    parser.add_argument(
        "--pcd-folder",
        type=str,
        default="pcd",
        help="Folder name under data root containing PCD files",
    )
    parser.add_argument(
        "--image-folder",
        type=str,
        default="image_a6",
        help="Folder name under data root containing camera images",
    )
    parser.add_argument(
        "--filename",
        type=str,
        default="0000000931",
        help="Base filename (without extension) for both PCD and image",
    )
    parser.add_argument(
        "--camera-name",
        type=str,
        default="a6",
        help="Camera name defined in calibration data",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=50.0,
        help="Maximum distance for color mapping (meters)",
    )
    parser.add_argument(
        "--point-radius",
        type=int,
        default=3,
        help="Radius (pixels) for drawing projected points",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="output",
        help="Directory to save projected image",
    )
    return parser.parse_args()


def main():
    """
    PCD를 이미지에 투영하는 메인 함수
    
    데이터 경로 구조:
        synchronized_data_pangyo_optimized/
        ├── pcd/            (PCD 파일)
        │   └── 0000050000.pcd
        └── img/            (RGB 이미지)
            └── 0000050000.jpg
    """
    
    # ===== 설정 =====
    args = parse_args()
    data_root = Path(args.data_root)
    pcd_filename = args.filename
    camera_name = args.camera_name
    max_distance = args.max_distance
    point_radius = args.point_radius
    
    # ===== 경로 설정 =====
    pcd_path = data_root / args.pcd_folder / f"{pcd_filename}.pcd"
    image_path = data_root / args.image_folder / f"{pcd_filename}.jpg"
    output_path = Path(args.output_dir) / f"{pcd_filename}_projected.jpg"
    
    print("=" * 70)
    print("PCD를 이미지에 투영하기")
    print("=" * 70)
    
    # ===== 파일 확인 =====
    print(f"\n[1] 파일 확인")
    if not pcd_path.exists():
        print(f"  ❌ PCD 파일을 찾을 수 없습니다: {pcd_path}")
        print(f"  💡 경로를 확인하세요")
        return
    print(f"  ✓ PCD: {pcd_path}")
    
    if not image_path.exists():
        print(f"  ❌ 이미지 파일을 찾을 수 없습니다: {image_path}")
        print(f"  💡 경로를 확인하세요")
        return
    print(f"  ✓ 이미지: {image_path}")
    
    # ===== PCD 로드 =====
    print(f"\n[2] PCD 파일 로드")
    try:
        cloud_xyz = load_pcd_xyz(pcd_path)
        cloud_xyz = ensure_positive_lidar(cloud_xyz)
        print(f"  ✓ 로드 완료: {cloud_xyz.shape[0]} 포인트")
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return
    
    # ===== 이미지 로드 =====
    print(f"\n[3] 이미지 로드")
    image = cv2.imread(str(image_path))
    if image is None:
        print(f"  ❌ 이미지를 로드할 수 없습니다")
        return
    h, w = image.shape[:2]
    print(f"  ✓ 로드 완료: {w}×{h}")
    
    # ===== 보정 데이터 설정 =====
    print(f"\n[4] 카메라 보정 설정")
    calib_db = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD)
    camera_info = calib_db.get(camera_name)
    camera_info["model"].set_image_size((w, h))
    print(f"  ✓ 카메라: {camera_name}")
    print(f"  ✓ Fisheye 모델")
    
    # ===== 투영 =====
    print(f"\n[5] 포인트 클라우드 투영")
    output_image, in_front, on_image = project_cloud_to_image(
        cloud_xyz=cloud_xyz,
        image=image,
        calib_db=calib_db,
        camera_name=camera_name,
        max_distance=max_distance,
        point_radius=point_radius
    )
    
    # ===== 결과 저장 =====
    print(f"\n[6] 결과 저장")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), output_image)
    print(f"  ✓ 저장 완료: {output_path}")
    
    # ===== 통계 =====
    print(f"\n[7] 통계")
    print(f"  총 포인트: {cloud_xyz.shape[0]:,}개")
    print(f"  카메라 앞: {in_front:,}개 ({100*in_front/cloud_xyz.shape[0]:.1f}%)")
    print(f"  이미지 범위: {on_image:,}개 ({100*on_image/cloud_xyz.shape[0]:.1f}%)")
    
    # ===== 3D 시각화 선택 =====
    print(f"\n[8] 추가 시각화")
    response = input("  투영 정보를 포함한 3D 시각화를 하시겠습니까? (y/n): ").lower().strip()
    if response == 'y':
        visualize_pcd_3d_with_projection(
            pcd_path=pcd_path,
            image_path=image_path,
            calib_db=calib_db,
            camera_name=camera_name,
            max_distance=max_distance
        )
    
    print("\n" + "=" * 70)
    print("완료!")
    print("=" * 70)


if __name__ == "__main__":
    main()
