"""
PCD를 이미지에 투영하는 심플한 도구
- DEFAULT_LIDAR_TO_CAM_v3 행렬만 사용
- 추가 변환 없음
"""

import argparse
import math
import numpy as np
import cv2
from pathlib import Path

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

# =============================================================================
# 캘리브레이션 데이터 (ref_calibration_data.py에서 가져옴)
# =============================================================================

# LiDAR → World (v3) - 새로운 값
LIDAR_TO_WORLD_v3 = np.array([
    [0.993292,    -0.10137,   -0.055641,    0.03384],
    [0.10098,      0.99484,  -0.00977845, -0.00561394],
    [0.0563451,  0.00409421,   0.998403,    0.749149],
    [0.,           0.,          0.,          1.]
])

# Extrinsic (Rodrigues 형식: tx, ty, tz, rx, ry, rz)
EXTRINSIC_RODRIGUES = [0.119933, -0.129544, -0.54216, -0.0333289, -0.166123, -0.0830659]

def rodrigues_to_matrix(rvec_tvec):
    """Rodrigues 벡터를 4x4 변환 행렬로 변환 (integrated_pcd_depth_pipeline_newest.py 방식)"""
    tvec = np.array(rvec_tvec[0:3]).reshape(3, 1)
    rvec = np.array(rvec_tvec[3:6])
    R, _ = cv2.Rodrigues(rvec)
    
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 0:3] = R
    transform_matrix[0:3, 3:4] = tvec
    return transform_matrix

# Extrinsic 행렬 (World → Camera)
EXTRINSIC_MATRIX = rodrigues_to_matrix(EXTRINSIC_RODRIGUES)

# integrated_pcd_depth_pipeline_newest.py 방식:
# lidar_to_camera_transform = cam_extrinsic @ lidar_to_world
DEFAULT_LIDAR_TO_CAM_v3 = EXTRINSIC_MATRIX @ LIDAR_TO_WORLD_v3

# VADAS Fisheye intrinsic 파라미터
INTRINSIC = [-0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,  # k[0~6]: u2d
             1.0447, 0.0021, 44.9516, 2.48822]  # s, div, ux, uy


# =============================================================================
# PCD 로드
# =============================================================================

def load_pcd_xyz(pcd_path: Path) -> np.ndarray:
    """PCD 파일에서 XYZ 좌표만 추출"""
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
            xyz = np.stack([data['x'], data['y'], data['z']], axis=-1).astype(np.float32)
            return xyz

        # ASCII fallback
        points_list = []
        for line in f:
            parts = line.decode('utf-8', errors='ignore').strip().split()
            if len(parts) >= 3:
                points_list.append([float(parts[0]), float(parts[1]), float(parts[2])])
        return np.array(points_list, dtype=np.float32)


# =============================================================================
# VADAS Fisheye 투영
# =============================================================================

def poly_eval(coeffs, x):
    """Polynomial 평가 (Horner's method)"""
    res = 0.0
    for c in reversed(coeffs):
        res = res * x + c
    return res


def project_point_vadas(Xc, Yc, Zc, intrinsic, image_size):
    """
    VADAS Fisheye 카메라 모델로 3D 점을 이미지 좌표로 투영
    
    카메라 좌표계 (VADAS):
    - +X: 전방 (카메라가 바라보는 방향)
    - +Y: 오른쪽
    - +Z: 아래쪽
    """
    k = intrinsic[0:7]   # Polynomial 계수
    s = intrinsic[7]     # Focal length scale
    div = intrinsic[8]   # Normalization divisor
    ux = intrinsic[9]    # Principal point X offset
    uy = intrinsic[10]   # Principal point Y offset
    
    w, h = image_size
    
    # 전방축이 음수면 카메라 뒤쪽 → 투영 불가
    # if Xc <= 0:
    #     return 0, 0, False
    
    # 극좌표 변환 (integrated_pcd_depth_pipeline_newest.py 방식)
    nx = -Yc
    ny = -Zc
    
    dist = math.hypot(nx, ny)
    if dist < 1e-10:
        dist = 1e-10
    
    cosPhi = nx / dist
    sinPhi = ny / dist
    
    # 각도 계산 (Xc가 forward 축)
    theta = math.atan2(dist, Xc)
    
    # Polynomial 입력
    xd = theta * s
    
    if abs(div) < 1e-9:
        return 0, 0, False
    
    # Polynomial 평가 및 정규화
    rd = poly_eval(k, xd) / div
    
    if math.isinf(rd) or math.isnan(rd):
        return 0, 0, False
    
    # 이미지 좌표 계산
    u = rd * cosPhi + ux + (w / 2)
    v = rd * sinPhi + uy + (h / 2)
    
    return int(round(u)), int(round(v)), True


# =============================================================================
# 메인 투영 함수
# =============================================================================

def get_color_from_distance(distance: float, max_distance: float = 50.0):
    """거리 기반 Jet colormap (BGR)"""
    normalized = min(distance / max_distance, 1.0)
    
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


def project_pcd_to_image(pcd_path: Path, image_path: Path, output_path: Path):
    """PCD를 이미지에 투영"""
    
    print("=" * 70)
    print("PCD → 이미지 투영 (DEFAULT_LIDAR_TO_CAM_v3 사용)")
    print("=" * 70)
    
    # [1] PCD 로드
    print(f"\n[1] PCD 로드")
    print(f"  경로: {pcd_path}")
    cloud_xyz = load_pcd_xyz(pcd_path)
    print(f"  ✓ 로드 완료: {cloud_xyz.shape[0]} 포인트")
    
    # [2] 이미지 로드
    print(f"\n[2] 이미지 로드")
    print(f"  경로: {image_path}")
    image = cv2.imread(str(image_path))
    if image is None:
        print("  ❌ 이미지를 로드할 수 없습니다")
        return
    h, w = image.shape[:2]
    print(f"  ✓ 로드 완료: {w}×{h}")
    
    # [3] LiDAR → Camera 변환
    print(f"\n[3] 좌표 변환 (LiDAR → Camera)")
    print(f"  변환 행렬: DEFAULT_LIDAR_TO_CAM_v3")
    
    # Homogeneous 좌표
    cloud_hom = np.hstack([cloud_xyz, np.ones((cloud_xyz.shape[0], 1))])
    
    # [FIX] LiDAR 좌표에서 X > 0인 포인트만 투영 (전방 포인트)
    forward_mask = cloud_xyz[:, 0] > 0
    cloud_hom = cloud_hom[forward_mask]
    print(f"  LiDAR X > 0 필터: {forward_mask.sum()}개 / {len(cloud_xyz)}개")
    
    # 변환 적용
    cam_pts_hom = (DEFAULT_LIDAR_TO_CAM_v3 @ cloud_hom.T).T
    cam_pts = cam_pts_hom[:, :3]
    
    print(f"  ✓ 변환 완료")
    print(f"  카메라 좌표 범위:")
    print(f"    Xc: {cam_pts[:,0].min():.2f} ~ {cam_pts[:,0].max():.2f}")
    print(f"    Yc: {cam_pts[:,1].min():.2f} ~ {cam_pts[:,1].max():.2f}")
    print(f"    Zc: {cam_pts[:,2].min():.2f} ~ {cam_pts[:,2].max():.2f}")
    
    # [4] 투영
    print(f"\n[4] 이미지 투영 (거리 기반 컬러맵)")
    output_image = image.copy()
    
    in_front_count = 0
    on_image_count = 0
    projection_mask = np.zeros(len(cam_pts), dtype=bool)
    
    for idx, (Xc, Yc, Zc) in enumerate(cam_pts):
        
        in_front_count += 1
        
        u, v, valid = project_point_vadas(Xc, Yc, Zc, INTRINSIC, (w, h))
        
        if valid and 0 <= u < w and 0 <= v < h:
            on_image_count += 1
            projection_mask[idx] = True
            # 거리 기반 컬러맵 (Xc = 전방 거리)
            color = get_color_from_distance(Xc, max_distance=50.0)
            cv2.circle(output_image, (u, v), 2, color, -1)
    
    print(f"  카메라 앞: {in_front_count}개")
    print(f"  이미지 범위: {on_image_count}개")
    print(f"  컬러맵: 파란색(가까움) → 초록색 → 빨간색(멀리)")
    
    # [5] 결과 저장
    print(f"\n[5] 결과 저장")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), output_image)
    print(f"  ✓ 저장: {output_path}")
    
    # [6] 3D 시각화 (옵션)
    if HAS_OPEN3D:
        print(f"\n[6] 3D 시각화")
        response = input("  3D 시각화를 하시겠습니까? (y/n): ").lower().strip()
        if response == 'y':
            # 필터된 포인트만 시각화 (X > 0)
            filtered_xyz = cloud_xyz[forward_mask]
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(filtered_xyz)
            
            # 색상: 투영된 점만 빨간색, 나머지 회색
            colors = np.full((len(filtered_xyz), 3), 0.4)
            colors[projection_mask] = [1.0, 0.0, 0.0]
            pcd.colors = o3d.utility.Vector3dVector(colors)
            
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)
            
            print("  🔴 빨간색 = 투영된 포인트")
            print("  ⚪ 회색 = 투영되지 않은 포인트 (X > 0만 표시)")
            
            o3d.visualization.draw_geometries(
                [pcd, coord_frame],
                window_name="PCD Projection",
                width=1200,
                height=800
            )
    
    print("\n" + "=" * 70)
    print("완료!")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Simple PCD to Image Projection")
    parser.add_argument(
        "--pcd",
        type=str,
        default="synchronized_data_pangyo_optimized/pcd/0000050000.pcd",
        help="PCD 파일 경로",
    )
    parser.add_argument(
        "--image",
        type=str,
        default="synchronized_data_pangyo_optimized/img/0000050000.jpg",
        help="이미지 파일 경로",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="output/projected_v3.jpg",
        help="출력 파일 경로",
    )
    args = parser.parse_args()
    
    pcd_path = Path(args.pcd)
    image_path = Path(args.image)
    output_path = Path(args.output)
    
    if not pcd_path.exists():
        print(f"❌ PCD 파일을 찾을 수 없습니다: {pcd_path}")
        return
    
    if not image_path.exists():
        print(f"❌ 이미지 파일을 찾을 수 없습니다: {image_path}")
        return
    
    project_pcd_to_image(pcd_path, image_path, output_path)


if __name__ == "__main__":
    main()
