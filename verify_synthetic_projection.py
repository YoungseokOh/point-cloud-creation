"""
원본 포인트와 Synthetic 포인트를 다른 색으로 이미지에 투영하여 시각적 검증
- 원본: 파란색 (거리 기반)
- Synthetic: 빨간색
"""

import argparse
import math
import numpy as np
import cv2
from pathlib import Path

# =============================================================================
# 캘리브레이션 데이터
# =============================================================================

LIDAR_TO_WORLD_v3 = np.array([
    [0.993292,    -0.10137,   -0.055641,    0.03384],
    [0.10098,      0.99484,  -0.00977845, -0.00561394],
    [0.0563451,  0.00409421,   0.998403,    0.749149],
    [0.,           0.,          0.,          1.]
])

EXTRINSIC_RODRIGUES = [0.119933, -0.129544, -0.54216, -0.0333289, -0.166123, -0.0830659]

def rodrigues_to_matrix(rvec_tvec):
    tvec = np.array(rvec_tvec[0:3]).reshape(3, 1)
    rvec = np.array(rvec_tvec[3:6])
    R, _ = cv2.Rodrigues(rvec)
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 0:3] = R
    transform_matrix[0:3, 3:4] = tvec
    return transform_matrix

EXTRINSIC_MATRIX = rodrigues_to_matrix(EXTRINSIC_RODRIGUES)
DEFAULT_LIDAR_TO_CAM_v3 = EXTRINSIC_MATRIX @ LIDAR_TO_WORLD_v3

INTRINSIC = [-0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,
             1.0447, 0.0021, 44.9516, 2.48822]


# =============================================================================
# c_circles 생성 (간단 버전)
# =============================================================================

BLUE_DEPTH_Z = -0.749149  # [FIX] 새로운 좌표계 v3의 Z translation 값과 일치

def generate_c_circles(all_points_np, y_zero_band=0.01, num_radii=20, circle_segs=360):
    """Synthetic c_circles 포인트 생성"""
    if all_points_np is None or all_points_np.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    pts = all_points_np.astype(np.float64, copy=False)

    # Purple point 선택 (|y|<=band, x>0, 최소 거리)
    x_vals = pts[:, 0]
    y_vals = pts[:, 1]
    mask = (np.abs(y_vals) <= y_zero_band) & (x_vals > 0.0)
    idxs = np.where(mask)[0]
    if idxs.size == 0:
        print("  [c_circles] No purple point candidates")
        return np.zeros((0, 3), dtype=np.float64)

    cand = pts[idxs]
    dists = np.linalg.norm(cand, axis=1)
    pos_mask = dists > 0.0
    if not np.any(pos_mask):
        return np.zeros((0, 3), dtype=np.float64)

    pts_pos = cand[pos_mask]
    best_local = int(np.argmin(dists[pos_mask]))
    purple = pts_pos[best_local]
    print(f"  [c_circles] Purple point: {purple}")

    # Tilted basis 계산
    x_p, y_p, z_p = float(purple[0]), float(purple[1]), float(purple[2])
    b_len = float(math.hypot(x_p, y_p))
    center = np.array([0.0, 0.0, BLUE_DEPTH_Z], dtype=np.float64)
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

    slope_c = float((z_p - BLUE_DEPTH_Z) / max(b_len, eps))
    u_elev = u_b_xy + slope_c * z_hat
    u_elev /= max(np.linalg.norm(u_elev), eps)

    # Radii 생성
    t = np.linspace(0.0, 1.0, num_radii + 1, dtype=np.float64)[1:]
    radii = t * c_len

    kept_points = []
    ts = np.linspace(0.0, 2.0 * np.pi, circle_segs, dtype=np.float64)
    cos_t = np.cos(ts)[:, None]
    sin_t = np.sin(ts)[:, None]

    for r in radii:
        if abs(r - c_len) <= max(1e-9, 1e-6 * c_len):
            continue

        ring = center + r * (cos_t * u_elev + sin_t * u_tan)
        ring = ring[ring[:, 0] > 0.0]  # X > 0 유지
        if ring.size == 0:
            continue
        kept_points.append(ring)

    if not kept_points:
        return np.zeros((0, 3), dtype=np.float64)

    result = np.vstack(kept_points).astype(np.float64)
    print(f"  [c_circles] Generated {result.shape[0]} synthetic points")
    return result
    return result


# =============================================================================
# PCD 로드
# =============================================================================

def load_pcd_xyz(pcd_path):
    with open(pcd_path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header error")
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
                    raise ValueError(f"Unsupported TYPE: {typ}")

                if cnt == 1:
                    dtype_fields.append((field_name, dtype))
                else:
                    dtype_fields.append((field_name, (dtype, cnt)))

            dtype = np.dtype(dtype_fields)
            data = np.fromfile(f, dtype=dtype, count=num_points)
            xyz = np.stack([data['x'], data['y'], data['z']], axis=-1).astype(np.float32)
            return xyz

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
    
    nx = -Yc
    ny = -Zc
    
    dist = math.hypot(nx, ny)
    if dist < 1e-10:
        dist = 1e-10
    
    cosPhi = nx / dist
    sinPhi = ny / dist
    theta = math.atan2(dist, Xc)
    xd = theta * s
    
    if abs(div) < 1e-9:
        return 0, 0, False
    
    rd = poly_eval(k, xd) / div
    
    if math.isinf(rd) or math.isnan(rd):
        return 0, 0, False
    
    u = rd * cosPhi + ux + (w / 2)
    v = rd * sinPhi + uy + (h / 2)
    
    return int(round(u)), int(round(v)), True


# =============================================================================
# 메인
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="원본 vs Synthetic 포인트 시각적 검증")
    parser.add_argument("--pcd", type=str, required=True, help="원본 PCD 파일 경로")
    parser.add_argument("--image", type=str, required=True, help="이미지 파일 경로")
    parser.add_argument("--output", type=str, default="output/verify_synthetic.jpg", help="출력 파일 경로")
    args = parser.parse_args()

    pcd_path = Path(args.pcd)
    image_path = Path(args.image)
    output_path = Path(args.output)

    if not pcd_path.exists():
        print(f"❌ PCD 파일 없음: {pcd_path}")
        return
    if not image_path.exists():
        print(f"❌ 이미지 파일 없음: {image_path}")
        return

    print("=" * 70)
    print("원본 vs Synthetic 포인트 시각적 검증")
    print("=" * 70)

    # [1] PCD 로드
    print(f"\n[1] PCD 로드: {pcd_path}")
    cloud_xyz = load_pcd_xyz(pcd_path)
    print(f"  전체 포인트: {cloud_xyz.shape[0]}")

    # X > 0 필터 (전방)
    forward_mask = cloud_xyz[:, 0] > 0
    original_pts = cloud_xyz[forward_mask]
    print(f"  원본 (X > 0): {original_pts.shape[0]}")

    # [2] Synthetic 포인트 생성
    print(f"\n[2] Synthetic 포인트 생성 (c_circles)")
    synthetic_pts = generate_c_circles(original_pts)
    print(f"  Synthetic: {synthetic_pts.shape[0]}")

    # [3] 이미지 로드
    print(f"\n[3] 이미지 로드: {image_path}")
    image = cv2.imread(str(image_path))
    if image is None:
        print("  ❌ 이미지 로드 실패")
        return
    h, w = image.shape[:2]
    print(f"  크기: {w}×{h}")

    # [4] 좌표 변환
    print(f"\n[4] 좌표 변환 (LiDAR → Camera)")
    
    # 원본
    orig_hom = np.hstack([original_pts, np.ones((original_pts.shape[0], 1))])
    orig_cam = (DEFAULT_LIDAR_TO_CAM_v3 @ orig_hom.T).T[:, :3]

    # Synthetic
    if synthetic_pts.shape[0] > 0:
        synth_hom = np.hstack([synthetic_pts, np.ones((synthetic_pts.shape[0], 1))])
        synth_cam = (DEFAULT_LIDAR_TO_CAM_v3 @ synth_hom.T).T[:, :3]
    else:
        synth_cam = np.empty((0, 3))

    # [5] 투영
    print(f"\n[5] 이미지 투영")
    output_image = image.copy()
    image_size = (w, h)

    # 원본 투영 (파란색 계열 - 거리 기반)
    orig_count = 0
    for i in range(orig_cam.shape[0]):
        Xc, Yc, Zc = orig_cam[i]
        if Xc <= 0:
            continue
        u, v, valid = project_point_vadas(Xc, Yc, Zc, INTRINSIC, image_size)
        if valid and 0 <= u < w and 0 <= v < h:
            # 거리 기반 색상 (파란색 → 초록색)
            dist = Xc
            norm_dist = min(dist / 30.0, 1.0)
            b = int(255 * (1 - norm_dist))
            g = int(255 * norm_dist)
            cv2.circle(output_image, (u, v), 2, (b, g, 0), -1)  # BGR
            orig_count += 1

    # Synthetic 투영 (빨간색)
    synth_count = 0
    for i in range(synth_cam.shape[0]):
        Xc, Yc, Zc = synth_cam[i]
        if Xc <= 0:
            continue
        u, v, valid = project_point_vadas(Xc, Yc, Zc, INTRINSIC, image_size)
        if valid and 0 <= u < w and 0 <= v < h:
            cv2.circle(output_image, (u, v), 3, (0, 0, 255), -1)  # 빨간색 (BGR)
            synth_count += 1

    print(f"  원본 투영: {orig_count}")
    print(f"  Synthetic 투영: {synth_count}")

    # [6] 저장
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), output_image)
    print(f"\n[6] 저장: {output_path}")
    print(f"  - 파란색/초록색: 원본 포인트 (거리 기반)")
    print(f"  - 빨간색: Synthetic 포인트")

    print("\n" + "=" * 70)
    print("완료!")
    print("=" * 70)


if __name__ == "__main__":
    main()
