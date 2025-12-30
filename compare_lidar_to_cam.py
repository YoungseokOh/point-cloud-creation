"""
두 가지 방법으로 LiDAR → Camera 변환 비교:
1. DEFAULT_LIDAR_TO_CAM_v3 (미리 계산된 행렬)
2. Extrinsic @ LiDAR_to_World (직접 계산)
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
# 캘리브레이션 데이터
# =============================================================================

# 미리 계산된 LiDAR → Camera 행렬
DEFAULT_LIDAR_TO_CAM_v3 = np.array([
    [ 0.98280272,  0.08533396, -0.16375873,  0.119933  ],
    [-0.07981367,  0.99600649,  0.04001059, -0.129544  ],
    [ 0.16651902, -0.02625233,  0.98568871, -0.54216   ],
    [ 0.,          0.,          0.,          1.        ]
])

# LiDAR → World 행렬
DEFAULT_LIDAR_TO_WORLD_v3 = np.array([
    [-0.99856,  -0.00901632,  -0.052883,   0.0539789],
    [0.00872567,  -0.999946,  0.00572437,  0.0837919],
    [-0.0529318, 0.00525468,   0.998584,   0.737086],
    [0.,         0.,          0.,           1.       ]
])

# Extrinsic (Rodrigues 형식: tx, ty, tz, rx, ry, rz)
EXTRINSIC_RODRIGUES = [0.119933, -0.129544, -0.54216, -0.0333289, -0.166123, -0.0830659]

# VADAS Fisheye intrinsic 파라미터
INTRINSIC = [-0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,
             1.0447, 0.0021, 44.9516, 2.48822]


def rodrigues_to_matrix(rodrigues_vec):
    """Rodrigues 벡터를 4x4 변환 행렬로 변환"""
    tvec = np.array(rodrigues_vec[:3], dtype=np.float32)
    rvec = np.array(rodrigues_vec[3:6], dtype=np.float32)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


# Extrinsic 행렬 (World → Camera)
EXTRINSIC_MATRIX = rodrigues_to_matrix(EXTRINSIC_RODRIGUES)

# 직접 계산: Extrinsic @ LiDAR_to_World
LIDAR_TO_CAM_COMPUTED = EXTRINSIC_MATRIX @ DEFAULT_LIDAR_TO_WORLD_v3


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
        fields, sizes, types, counts = [], [], [], []

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
                dtype_fields.append((field_name, dtype) if cnt == 1 else (field_name, (dtype, cnt)))
            dtype = np.dtype(dtype_fields)
            data = np.fromfile(f, dtype=dtype, count=num_points)
            return np.stack([data['x'], data['y'], data['z']], axis=-1).astype(np.float32)

        # ASCII
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
    res = 0.0
    for c in reversed(coeffs):
        res = res * x + c
    return res


def project_point_vadas(Xc, Yc, Zc, intrinsic, image_size):
    k = intrinsic[0:7]
    s = intrinsic[7]
    div = intrinsic[8]
    ux = intrinsic[9]
    uy = intrinsic[10]
    w, h = image_size
    
    if Xc <= 0:
        return 0, 0, False
    
    forward = Xc
    horiz = -Yc
    vert = -Zc
    
    dist = math.hypot(horiz, vert)
    if dist < 1e-10:
        dist = 1e-10
    
    cosPhi = horiz / dist
    sinPhi = vert / dist
    theta = math.atan2(dist, forward)
    xd = theta * s
    
    if abs(div) < 1e-9:
        return 0, 0, False
    
    rd = poly_eval(k, xd) / div
    
    if math.isinf(rd) or math.isnan(rd):
        return 0, 0, False
    
    u = rd * cosPhi + ux + (w / 2)
    v = rd * sinPhi + uy + (h / 2)
    
    return int(round(u)), int(round(v)), True


def get_color_from_distance(distance: float, max_distance: float = 50.0):
    """거리 기반 Jet colormap (BGR)"""
    normalized = min(distance / max_distance, 1.0)
    
    if normalized < 0.25:
        r, g, b = 0, int(255 * (normalized / 0.25)), 255
    elif normalized < 0.5:
        r, g, b = 0, 255, int(255 * (1 - (normalized - 0.25) / 0.25))
    elif normalized < 0.75:
        r, g, b = int(255 * ((normalized - 0.5) / 0.25)), 255, 0
    else:
        r, g, b = 255, int(255 * (1 - (normalized - 0.75) / 0.25)), 0
    
    return (b, g, r)


# =============================================================================
# 투영 함수
# =============================================================================

def project_with_matrix(cloud_xyz, image, lidar_to_cam, name):
    """특정 변환 행렬로 투영"""
    h, w = image.shape[:2]
    output = image.copy()
    
    # 변환
    cloud_hom = np.hstack([cloud_xyz, np.ones((cloud_xyz.shape[0], 1))])
    cam_pts = (lidar_to_cam @ cloud_hom.T).T[:, :3]
    
    count = 0
    for Xc, Yc, Zc in cam_pts:
        if Xc <= 0:
            continue
        u, v, valid = project_point_vadas(Xc, Yc, Zc, INTRINSIC, (w, h))
        if valid and 0 <= u < w and 0 <= v < h:
            count += 1
            color = get_color_from_distance(Xc, 50.0)
            cv2.circle(output, (u, v), 2, color, -1)
    
    # 라벨 추가
    cv2.putText(output, name, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)
    cv2.putText(output, name, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)
    cv2.putText(output, f"Points: {count}", (50, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)
    cv2.putText(output, f"Points: {count}", (50, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 2)
    
    return output, count


def compare_projections(pcd_path: Path, image_path: Path, output_dir: Path):
    """두 가지 변환 행렬로 투영 비교"""
    
    print("=" * 70)
    print("LiDAR → Camera 변환 행렬 비교")
    print("=" * 70)
    
    # 행렬 출력
    print("\n[1] 행렬 비교")
    print("\n  === DEFAULT_LIDAR_TO_CAM_v3 (미리 계산됨) ===")
    print(DEFAULT_LIDAR_TO_CAM_v3)
    
    print("\n  === Extrinsic @ LiDAR_to_World (직접 계산) ===")
    print(LIDAR_TO_CAM_COMPUTED)
    
    print("\n  === 차이 (Computed - v3) ===")
    diff = LIDAR_TO_CAM_COMPUTED - DEFAULT_LIDAR_TO_CAM_v3
    print(diff)
    print(f"\n  최대 차이: {np.abs(diff).max():.6f}")
    
    # PCD 로드
    print(f"\n[2] PCD 로드: {pcd_path}")
    cloud_xyz = load_pcd_xyz(pcd_path)
    print(f"  ✓ {cloud_xyz.shape[0]} 포인트")
    
    # 이미지 로드
    print(f"\n[3] 이미지 로드: {image_path}")
    image = cv2.imread(str(image_path))
    h, w = image.shape[:2]
    print(f"  ✓ {w}×{h}")
    
    # 투영
    print(f"\n[4] 투영 비교")
    
    output_v3, count_v3 = project_with_matrix(
        cloud_xyz, image, DEFAULT_LIDAR_TO_CAM_v3, "DEFAULT_LIDAR_TO_CAM_v3"
    )
    print(f"  v3 (미리 계산): {count_v3} 포인트")
    
    output_computed, count_computed = project_with_matrix(
        cloud_xyz, image, LIDAR_TO_CAM_COMPUTED, "Extrinsic @ LiDAR_to_World"
    )
    print(f"  Computed (직접 계산): {count_computed} 포인트")
    
    # 나란히 비교 이미지 생성
    combined = np.hstack([output_v3, output_computed])
    
    # 저장
    output_dir.mkdir(parents=True, exist_ok=True)
    
    cv2.imwrite(str(output_dir / "compare_v3.jpg"), output_v3)
    cv2.imwrite(str(output_dir / "compare_computed.jpg"), output_computed)
    cv2.imwrite(str(output_dir / "compare_side_by_side.jpg"), combined)
    
    print(f"\n[5] 결과 저장")
    print(f"  ✓ {output_dir / 'compare_v3.jpg'}")
    print(f"  ✓ {output_dir / 'compare_computed.jpg'}")
    print(f"  ✓ {output_dir / 'compare_side_by_side.jpg'}")
    
    print("\n" + "=" * 70)
    print("완료!")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Compare LiDAR to Camera transformations")
    parser.add_argument("--pcd", type=str, default="synchronized_data_pangyo_optimized/pcd/0000050000.pcd")
    parser.add_argument("--image", type=str, default="synchronized_data_pangyo_optimized/img/0000050000.jpg")
    parser.add_argument("--output", type=str, default="output/compare_lidar_to_cam")
    args = parser.parse_args()
    
    pcd_path = Path(args.pcd)
    image_path = Path(args.image)
    output_dir = Path(args.output)
    
    if not pcd_path.exists():
        print(f"❌ PCD 파일을 찾을 수 없습니다: {pcd_path}")
        return
    
    if not image_path.exists():
        print(f"❌ 이미지 파일을 찾을 수 없습니다: {image_path}")
        return
    
    compare_projections(pcd_path, image_path, output_dir)


if __name__ == "__main__":
    main()
