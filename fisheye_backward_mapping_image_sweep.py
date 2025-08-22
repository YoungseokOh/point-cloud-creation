"""
Fisheye Backward Mapping (with FOV Auto-Sweep + Visualization)

Purpose:
- Take a source 2D image (e.g., Depth Anything v2 output, or any rectified image),
  and synthesize a fisheye-view image using backward mapping (hole-free).
- Optionally sweep candidate horizontal FOVs for the (unknown) source pinhole K_src,
  score each, and visualize results for quick selection.

Core assumptions:
- Fisheye and pinhole share the same camera center (t = 0). Rotation may differ.
  (Translation t ≠ 0 is not supported from a single image.)

Usage (examples):
  python fisheye_backward_mapping_image_sweep.py \
    --image_path ./example_image.png \
    --out_dir ./out \
    --src_fov_list 60,70,80,90 \
    --dst_size 1024x1024

  # If you don't have an image, the script will generate a synthetic one.
"""

import os
import argparse
import math
import json
import numpy as np
import cv2 as cv
from typing import Tuple, List, Optional
from scipy.ndimage import map_coordinates

# 프로젝트 내 모듈 import
from ref.ref_calibration_data import DEFAULT_CALIB, DEFAULT_LIDAR_TO_CAM
from ref.ref_camera_lidar_projector import VADASFisheyeCameraModel


# ----------------------
# Utilities
# ----------------------
def parse_size(s: str) -> Tuple[int, int]:
    if "x" not in s:
        raise ValueError("--dst_size must be like 1024x1024")
    w, h = s.split("x")
    return int(w), int(h)





def percentile(x: np.ndarray, q: float) -> float:
    return float(np.percentile(x.ravel(), q))


def compute_rotation_between_vectors(v1, v2):
    """두 단위 벡터 사이의 회전 행렬 계산 (Rodrigues 공식 사용)"""
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    
    cross = np.cross(v1, v2)
    dot = np.dot(v1, v2)
    
    if np.allclose(cross, 0):  # 평행한 벡터
        return np.eye(3) if dot > 0 else -np.eye(3)
    
    if np.linalg.norm(cross) < 1e-8:  # 거의 평행한 벡터
        return np.eye(3) if dot > 0 else -np.eye(3)
    
    skew_symmetric = np.array([[0, -cross[2], cross[1]],
                               [cross[2], 0, -cross[0]],
                               [-cross[1], cross[0], 0]])
    
    R = np.eye(3) + skew_symmetric + skew_symmetric @ skew_symmetric * (1 - dot) / (np.linalg.norm(cross) ** 2)
    return R


# ----------------------
# VADAS Camera & Mapping Helpers (Vectorized)
# ----------------------
def _poly_eval(coeffs: List[float], x: np.ndarray) -> np.ndarray:
    """다항식 계산 (벡터화)"""
    result = np.zeros_like(x, dtype=np.float64)
    for c in reversed(coeffs):
        result = result * x + c
    return result

def _poly_deriv_eval(coeffs: List[float], x: np.ndarray) -> np.ndarray:
    """다항식 도함수 계산 (벡터화)"""
    if len(coeffs) <= 1:
        return np.zeros_like(x, dtype=np.float64)
    
    result = np.zeros_like(x, dtype=np.float64)
    for i, c in enumerate(reversed(coeffs[1:]), 1):
        result = result * x + c * i
    return result


def _estimate_theta_from_rd(vadas_model: VADASFisheyeCameraModel, rd_array: np.ndarray, max_iter: int = 10) -> np.ndarray:
    """rd 배열에서 theta를 추정하는 반복 방법 (벡터화)"""
    if abs(vadas_model.div) < 1e-9:
        return np.full_like(rd_array, np.nan)

    # 초기값: theta ≈ rd / s (작은 각도 근사)
    theta = np.where(vadas_model.s != 0, rd_array / vadas_model.s, rd_array)
    theta = np.clip(theta, 0, np.pi * 0.9)  # 안전한 범위로 제한
    
    converged_mask = np.zeros_like(rd_array, dtype=bool)

    for iteration in range(max_iter):
        xd = theta * vadas_model.s
        poly_val = _poly_eval(vadas_model.k, xd)
        poly_deriv = _poly_deriv_eval(vadas_model.k, xd)
        
        # 안정성 체크
        unstable_deriv_mask = np.abs(poly_deriv * vadas_model.s) < 1e-9
        
        rd_pred = poly_val / vadas_model.div
        error = rd_pred - rd_array
        
        current_converged = np.abs(error) < 1e-6
        converged_mask = converged_mask | current_converged | unstable_deriv_mask

        update_mask = ~converged_mask
        if not np.any(update_mask):
            break

        # Newton-Raphson 업데이트 (스텝 크기 제한)
        theta_update = np.zeros_like(theta)
        step_size = error[update_mask] / (poly_deriv[update_mask] * vadas_model.s / vadas_model.div)
        step_size = np.clip(step_size, -0.1, 0.1)  # 스텝 크기 제한
        theta_update[update_mask] = step_size
        
        theta = theta - theta_update
        theta = np.clip(theta, 0, np.pi * 0.9)

    # 유효하지 않은 각도 필터링
    theta = np.where((theta >= 0) & (theta <= np.pi * 0.9), theta, np.nan)
    return theta

def rotation_matrix_x(angle_rad):
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def rotation_matrix_y(angle_rad):
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

def vadas_undistort_points_vectorized(pts_2d: np.ndarray, vadas_model: VADASFisheyeCameraModel) -> np.ndarray:
    """
    VADAS Fisheye 모델을 사용하여 2D 이미지 포인트를 3D 단위 광선으로 역투영.
    실제 VADAS project_point 구현을 기반으로 역변환 수행.
    """
    u_coords = pts_2d[:, 0]
    v_coords = pts_2d[:, 1]

    img_w_half = vadas_model.image_size[0] / 2
    img_h_half = vadas_model.image_size[1] / 2
    
    # VADAS project_point의 역과정:
    # u = rd * cosPhi + self.ux + img_w_half
    # v = rd * sinPhi + self.uy + img_h_half
    u_centered = u_coords - img_w_half - vadas_model.ux
    v_centered = v_coords - img_h_half - vadas_model.uy
    
    rd = np.sqrt(u_centered**2 + v_centered**2)
    
    # rd가 0에 가까운 경우 처리
    zero_rd_mask = rd < 1e-6
    
    cosPhi = np.zeros_like(rd)
    sinPhi = np.zeros_like(rd)
    non_zero_rd_mask = ~zero_rd_mask
    cosPhi[non_zero_rd_mask] = u_centered[non_zero_rd_mask] / rd[non_zero_rd_mask]
    sinPhi[non_zero_rd_mask] = v_centered[non_zero_rd_mask] / rd[non_zero_rd_mask]
    
    # rd에서 theta 추정
    theta = _estimate_theta_from_rd(vadas_model, rd)
    valid_theta_mask = ~np.isnan(theta)
    
    # VADAS 좌표계에서 3D 단위 광선 계산
    # project_point에서: nx = -Yc, ny = -Zc, theta = atan2(dist, Xc)
    # 역변환: 
    # - dist = sin(theta), Xc = cos(theta)
    # - nx = rd*cosPhi = -Yc → Yc = -rd*cosPhi
    # - ny = rd*sinPhi = -Zc → Zc = -rd*sinPhi
    
    Xc_unit = np.cos(theta)  # forward component
    dist_unit = np.sin(theta)
    
    # 정규화를 위해 dist_unit을 사용
    Yc_unit = -dist_unit * cosPhi  # right component  
    Zc_unit = -dist_unit * sinPhi  # down component
    
    # 결과 배열 초기화
    unit_rays = np.full((len(u_coords), 3), np.nan, dtype=np.float64)

    # 유효한 포인트에 대해서만 값 할당
    final_valid_mask = valid_theta_mask & non_zero_rd_mask
    unit_rays[final_valid_mask, 0] = Xc_unit[final_valid_mask]
    unit_rays[final_valid_mask, 1] = Yc_unit[final_valid_mask]
    unit_rays[final_valid_mask, 2] = Zc_unit[final_valid_mask]

    # 광축 중심점 처리
    unit_rays[zero_rd_mask, 0] = 1.0  # X_forward = 1
    unit_rays[zero_rd_mask, 1] = 0.0  # Y_right = 0
    unit_rays[zero_rd_mask, 2] = 0.0  # Z_down = 0
    
    return unit_rays

def vadas_rays_to_std(rays_vadas: np.ndarray) -> np.ndarray:
    """
    VADAS 좌표계 [X_forward, Y_right, Z_down]를  
    표준 좌표계 [x_right, y_down, z_forward]로 변환
    """
    Xf = rays_vadas[..., 0]  # forward
    Yr = rays_vadas[..., 1]  # right  
    Zd = rays_vadas[..., 2]  # down
    
    # 좌표계 변환 (ref_camera_lidar_projector.py 참조)
    # VADAS: X=forward, Y=right, Z=down
    # 표준: x=right, y=down, z=forward
    x = -Yr   # Y_right -> x_right
    y = -Zd   # Z_down -> y_down
    z = Xf  # X_forward -> -z_forward (flipped to point forward)
    
    rays_std = np.stack([x, y, z], axis=-1)
    
    # 정규화
    n = np.linalg.norm(rays_std, axis=-1, keepdims=True) + 1e-12
    return rays_std / n

def debug_vadas_coordinate_system(vadas_model, image_size):
    """VADAS 좌표계 변환 검증 (강화된 버전)"""
    W, H = image_size
    
    print(f"[DEBUG] VADAS Model Parameters:")
    print(f"  Image size: {W}x{H}")
    print(f"  ux: {vadas_model.ux:.3f}, uy: {vadas_model.uy:.3f}")
    print(f"  s: {vadas_model.s:.6f}, div: {vadas_model.div:.6f}")
    print(f"  k coeffs: {[f'{k:.6f}' for k in vadas_model.k]}")
    
    # 더 많은 테스트 포인트
    test_points = np.array([
        [W/2, H/2],           # 중심점
        [W/2 + W/4, H/2],     # 우측
        [W/2 - W/4, H/2],     # 좌측  
        [W/2, H/2 - H/4],     # 상단
        [W/2, H/2 + H/4],     # 하단
        [W/4, H/4],           # 좌상단
        [3*W/4, 3*H/4],       # 우하단
    ])
    
    rays_vadas = vadas_undistort_points_vectorized(test_points, vadas_model)
    rays_std = vadas_rays_to_std(rays_vadas)
    
    print("[DEBUG] VADAS Coordinate System Validation:")
    labels = ["Center", "Right", "Left", "Top", "Bottom", "TopLeft", "BottomRight"]
    for i, (label, pt, ray_v, ray_s) in enumerate(zip(labels, test_points, rays_vadas, rays_std)):
        theta_deg = np.degrees(np.arccos(np.clip(ray_s[2], -1, 1)))
        print(f"  {label:10} {pt}: VADAS{ray_v} -> STD{ray_s} (θ={theta_deg:.1f}°)")


def fix_rotation_alignment(vadas_model, image_size):
    """회전 정렬 수정 - 실제 VADAS 중심점을 계산하여 사용"""
    W, H = image_size
    
    # 1. VADAS 모델에서 실제 중심점의 광선 계산
    center_point = np.array([[W/2, H/2]], dtype=np.float32)
    center_ray_vadas = vadas_undistort_points_vectorized(center_point, vadas_model)[0]
    center_ray_std = vadas_rays_to_std(center_ray_vadas.reshape(1, -1))[0]
    
    print(f"[DEBUG] Actual center ray calculation:")
    print(f"  Center point: [{W/2}, {H/2}]")
    print(f"  VADAS ray: {center_ray_vadas}")
    print(f"  STD ray: {center_ray_std}")
    
    # 2. 목표: 표준 좌표계에서 [0, 0, 1] (정면)으로 정렬
    target_ray = np.array([0, 0, 1])
    
    # 3. 두 벡터 사이의 회전 행렬 계산
    R_fe2src = compute_rotation_between_vectors(center_ray_std, target_ray)
    
    print(f"[INFO] Computed rotation matrix R_fe2src:\n{R_fe2src}")
    
    # 4. 회전 검증
    rotated_ray = R_fe2src @ center_ray_std
    print(f"[DEBUG] Verification - rotated center ray: {rotated_ray}")
    print(f"[DEBUG] Should be close to [0, 0, 1]: {np.allclose(rotated_ray, target_ray, atol=1e-3)}")
    
    return R_fe2src


# ----------------------
# Camera & Mapping
# ----------------------
def K_from_fov(W: int, H: int, fov_deg: float, cx=None, cy=None) -> np.ndarray:
    """Build a pinhole K from horizontal FOV (deg)."""
    # Cap FOV slightly below 180 to avoid tan(pi/2) issues
    if fov_deg >= 180.0:
        fov_deg = 179.9 # Practical limit for pinhole FOV

    theta = math.radians(fov_deg)
    fx = W / (2.0 * math.tan(theta / 2.0))
    fy = fx  # pixel aspect 1.0 assumed
    cx = (W / 2.0) if cx is None else cx
    cy = (H / 2.0) if cy is None else cy
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return K


def forward_map_depth_to_fisheye(image_src_depth: np.ndarray,
                               K_src: np.ndarray,
                               vadas_fish_model: VADASFisheyeCameraModel,
                               R_fe2src: np.ndarray,
                               dst_size: Tuple[int, int]) -> np.ndarray:
    W_src, H_src = image_src_depth.shape[1], image_src_depth.shape[0]
    W_out, H_out = dst_size

    # 출력 어안 깊이 맵 초기화 (구멍이 생길 수 있으므로 0으로 초기화)
    dst_depth_map = np.zeros((H_out, W_out), dtype=image_src_depth.dtype)
    dst_count_map = np.zeros((H_out, W_out), dtype=np.int32)  # 중복 투영 카운트

    fx, fy, cx, cy = K_src[0, 0], K_src[1, 1], K_src[0, 2], K_src[1, 2]

    print(f"[INFO] Forward mapping depth from {W_src}x{H_src} to fisheye {W_out}x{H_out}")

    # 소스 이미지의 모든 픽셀을 순회
    valid_projections = 0
    total_pixels = 0
    
    for v_src in range(H_src):
        if v_src % 100 == 0:  # 진행률 표시
            progress = v_src / H_src * 100
            print(f"[INFO] Processing row {v_src}/{H_src} ({progress:.1f}%)")
            
        for u_src in range(W_src):
            depth_val = image_src_depth[v_src, u_src]
            depth_val = float(depth_val) # 배열일 경우를 대비해 스칼라로 변환
            total_pixels += 1

            if depth_val <= 0: # 유효하지 않은 깊이 (0 또는 음수)는 건너뜀
                continue

            # 1) 소스 핀홀 카메라 좌표계에서 3D 점 계산
            X_src_cam = (u_src - cx) * depth_val / fx
            Y_src_cam = (v_src - cy) * depth_val / fy
            Z_src_cam = depth_val
            
            point_src_cam = np.array([X_src_cam, Y_src_cam, Z_src_cam])

            # 2) 소스 카메라 좌표계에서 어안 카메라 좌표계로 3D 점 변환
            # R_fe2src는 fisheye -> source 이므로, source -> fisheye는 R_fe2src.T
            point_fe_cam = R_fe2src.T @ point_src_cam

            # 3) 어안 카메라 모델을 사용하여 3D 점을 2D 어안 픽셀로 투영
            u_fe, v_fe, valid_projection = vadas_fish_model.project_point(
                point_fe_cam[0], point_fe_cam[1], point_fe_cam[2]
            )

            # 4) 유효한 투영이고 출력 이미지 범위 내에 있으면 깊이 값 할당
            if valid_projection & (0 <= u_fe < W_out) & (0 <= v_fe < H_out):
                # 정수 좌표로 반올림
                u_fe_int, v_fe_int = int(round(u_fe)), int(round(v_fe))
                
                if (0 <= u_fe_int < W_out) & (0 <= v_fe_int < H_out):
                    # 가중 평균 또는 최근접 깊이 선택
                    if dst_count_map[v_fe_int, u_fe_int] == 0:
                        dst_depth_map[v_fe_int, u_fe_int] = depth_val
                    else:
                        # 기존 깊이와 새 깊이의 평균 (또는 최소값 선택)
                        existing_depth = dst_depth_map[v_fe_int, u_fe_int]
                        dst_depth_map[v_fe_int, u_fe_int] = min(existing_depth, depth_val)
                    
                    dst_count_map[v_fe_int, u_fe_int] += 1
                    valid_projections += 1
    
    print(f"[INFO] Forward mapping completed:")
    print(f"  Total pixels: {total_pixels}")
    print(f"  Valid projections: {valid_projections}")
    print(f"  Non-zero pixels in output: {np.count_nonzero(dst_depth_map)}")
    
    return dst_depth_map


def backward_map_depth_to_fisheye(image_src_depth: np.ndarray,
                                K_src: np.ndarray,
                                vadas_fish_model: VADASFisheyeCameraModel,
                                R_fe2src: np.ndarray,
                                dst_size: Tuple[int, int]) -> np.ndarray:
    """
    Backward mapping: Fisheye 이미지의 각 픽셀에서 소스로 역투영하여 깊이값 가져오기
    구멍이 생기지 않는 방식 (각 출력 픽셀이 보장됨)
    
    Args:
        image_src_depth: 소스 깊이 맵 (H_src, W_src)
        K_src: 소스 카메라 intrinsic matrix
        vadas_fish_model: VADAS fisheye 카메라 모델
        R_fe2src: fisheye -> source 회전 행렬
        dst_size: 출력 크기 (W_out, H_out)
    
    Returns:
        dst_depth_map: Fisheye 깊이 맵 (H_out, W_out)
    """
    W_out, H_out = dst_size
    H_src, W_src = image_src_depth.shape
    
    print(f"[INFO] Backward mapping depth from {W_src}x{H_src} to fisheye {W_out}x{H_out}")
    
    # 1. Fisheye 이미지의 모든 픽셀 좌표 생성
    u_coords, v_coords = np.meshgrid(np.arange(W_out), np.arange(H_out))
    fisheye_pts = np.stack([u_coords.ravel(), v_coords.ravel()], axis=1).astype(np.float32)
    
    print(f"[INFO] Generated {len(fisheye_pts)} fisheye pixel coordinates")
    
    # 2. Fisheye 픽셀을 3D 광선으로 변환 (VADAS 좌표계)
    rays_vadas = vadas_undistort_points_vectorized(fisheye_pts, vadas_fish_model)
    
    # 3. VADAS 좌표계를 표준 좌표계로 변환
    rays_std = vadas_rays_to_std(rays_vadas)
    
    # 4. Fisheye 광선을 소스 카메라 좌표계로 회전
    # R_fe2src는 fisheye -> source이므로 그대로 사용
    rays_src_cam = (R_fe2src @ rays_std.T).T
    
    # 5. 유효한 광선 필터링 (카메라 앞쪽을 보는 광선만)
    valid_ray_mask = rays_src_cam[:, 2] > 1e-6  # Z > 0 (카메라 앞쪽)
    
    print(f"[INFO] Valid rays pointing forward: {np.sum(valid_ray_mask)}/{len(rays_src_cam)} ({np.sum(valid_ray_mask)/len(rays_src_cam):.3f})")
    
    # 6. 소스 카메라 좌표계에서 2D 픽셀로 투영
    fx, fy, cx, cy = K_src[0, 0], K_src[1, 1], K_src[0, 2], K_src[1, 2]
    
    # Perspective projection
    u_src = np.zeros(len(rays_src_cam))
    v_src = np.zeros(len(rays_src_cam))
    
    u_src[valid_ray_mask] = (fx * rays_src_cam[valid_ray_mask, 0] / rays_src_cam[valid_ray_mask, 2]) + cx
    v_src[valid_ray_mask] = (fy * rays_src_cam[valid_ray_mask, 1] / rays_src_cam[valid_ray_mask, 2]) + cy
    
    # 7. 소스 이미지 범위 내 체크
    in_bounds_mask = (u_src >= 0) & (u_src < W_src) & (v_src >= 0) & (v_src < H_src)
    final_valid_mask = valid_ray_mask & in_bounds_mask
    
    print(f"[INFO] Rays within source image bounds: {np.sum(final_valid_mask)}/{len(rays_src_cam)} ({np.sum(final_valid_mask)/len(rays_src_cam):.3f})")
    
    # 8. 유효한 좌표에 대해 bilinear interpolation으로 깊이값 샘플링
    coords = np.array([v_src[final_valid_mask], u_src[final_valid_mask]])
    
    # Bilinear interpolation (order=1)
    interpolated_depths = map_coordinates(
        image_src_depth, 
        coords, 
        order=1,  # bilinear
        mode='constant',  # 범위 밖은 0으로
        cval=0.0, 
        prefilter=False
    )
    
    # 9. 결과 조합
    dst_depth_map = np.zeros(W_out * H_out, dtype=image_src_depth.dtype)
    dst_depth_map[final_valid_mask] = interpolated_depths
    
    # 2D로 reshape
    dst_depth_map = dst_depth_map.reshape(H_out, W_out)
    
    # 10. 통계 정보 출력
    valid_output_pixels = np.sum(dst_depth_map > 0)
    total_output_pixels = W_out * H_out
    coverage_ratio = valid_output_pixels / total_output_pixels
    
    print(f"[INFO] Backward mapping completed:")
    print(f"  Output pixels with depth: {valid_output_pixels}/{total_output_pixels} ({coverage_ratio:.3f})")
    print(f"  Output depth range: {dst_depth_map[dst_depth_map > 0].min():.3f} - {dst_depth_map.max():.3f}")
    
    return dst_depth_map


# ----------------------
# Visualization
# ----------------------
def colorize_image(img: np.ndarray, hole_color="black") -> np.ndarray:
    """
    이미지를 시각화하기 위해 BGR로 변환 (필요시).
    그레이스케일 이미지의 경우 컬러맵을 적용.
    구멍(0값) 부분은 특별한 색으로 처리.
    
    Args:
        img: 입력 이미지 (2D depth map)
        hole_color: 구멍 부분의 색상 ("black", "white", "red", "green", "blue", "magenta")
    """
    if img.size == 0:
        return np.zeros((1, 1, 3), np.uint8)
    
    if img.ndim == 2:  # 그레이스케일 이미지 (깊이 맵으로 간주)
        # 0이 아닌 유효한 깊이 값만 고려하여 정규화
        valid_mask = img > 0 
        hole_mask = img == 0
        
        if np.any(valid_mask):
            min_val = img[valid_mask].min()
            max_val = img[valid_mask].max()
            if max_val - min_val > 1e-6:  # 0으로 나누는 것을 방지
                normalized_img = np.zeros_like(img, dtype=np.float32)
                normalized_img[valid_mask] = (img[valid_mask] - min_val) / (max_val - min_val)
            else:  # 모든 유효 픽셀 값이 동일한 경우
                normalized_img = np.zeros_like(img, dtype=np.float32)
                normalized_img[valid_mask] = 1.0
        else:  # 유효한 픽셀이 없는 경우
            normalized_img = np.zeros_like(img, dtype=np.float32)

        # 컬러맵 적용 (유효한 픽셀만)
        x8 = (np.clip(normalized_img, 0.0, 1.0) * 255.0).astype(np.uint8)
        colored_img = cv.applyColorMap(x8, cv.COLORMAP_JET)
        
        # 구멍 부분에 특별한 색상 적용
        hole_colors = {
            "black": (0, 0, 0),
            "white": (255, 255, 255), 
            "red": (0, 0, 255),        # BGR 순서
            "green": (0, 255, 0),
            "blue": (255, 0, 0),
            "magenta": (255, 0, 255),
            "cyan": (255, 255, 0),
            "yellow": (0, 255, 255),
            "gray": (128, 128, 128),
            "dark_gray": (64, 64, 64)
        }
        
        if hole_color in hole_colors:
            colored_img[hole_mask] = hole_colors[hole_color]
        else:
            # 기본값: 어두운 회색
            colored_img[hole_mask] = hole_colors["dark_gray"]
        
        return colored_img
    else:  # 컬러 이미지 (BGR)
        return img.astype(np.uint8)
    

def put_label(img: np.ndarray, text: str,
              org=(10, 28), font=cv.FONT_HERSHEY_SIMPLEX, scale=0.7,
              color=(255, 255, 255), thickness=2, lineType=cv.LINE_AA) -> np.ndarray:
    out = img.copy()
    cv.putText(out, text, org, font, scale, color, thickness, lineType)
    return out


# ----------------------
# Image IO
# ----------------------
def load_image_any(path: str) -> np.ndarray:
    """
    Load image from .npy / .npz / png / jpg etc.
    For depth maps, it attempts to load as grayscale.
    """
    ext = os.path.splitext(path)[1].lower()
    if ext == ".npy":
        img = np.load(path)
        if (img.ndim == 3) & (img.shape[2] == 3):
            img = img[:, :, 0]
    elif ext == ".npz":
        arr = np.load(path)
        # guess first array
        key = list(arr.keys())[0]
        img = arr[key]
        if (img.ndim == 3) & (img.shape[2] == 3):
            img = img[:, :, 0]
    else:
        # try image read (PNG, JPG, etc.)
        # 깊이 맵은 보통 1채널 그레이스케일이므로 IMREAD_GRAYSCALE로 로드 시도
        img = cv.imread(path, cv.IMREAD_GRAYSCALE) 
        if img is None:
            # 그레이스케일로 로드 실패 시, 컴러로 로드 후 첫 채널만 사용
            img = cv.imread(path, cv.IMREAD_UNCHANGED)
            if img is None:
                raise FileNotFoundError(f"Failed to read {path}")
            if img.ndim == 3: # 컬러 이미지인 경우 첫 채널만 사용 (깊이 맵 가정)
                img = img[:, :, 0]
            elif img.ndim == 4: # RGBA인 경우 RGB로 변환 후 첫 채널
                img = cv.cvtColor(img, cv.COLOR_RGBA2BGR)[:, :, 0]
    
    # 최종적으로 2D (H, W) 형태인지 확인
    if (img.ndim == 3) & (img.shape[2] == 1):
        img = img.squeeze()
    elif img.ndim != 2:
        raise ValueError(f"Loaded image has unexpected dimensions for a depth map: {img.shape}. Expected 2D.")

    return img.astype(np.float32) # float32로 변환하여 반환


# ----------------------
# Main Pipeline
# ----------------------
def visualize_vadas_depth_map_directly(vadas_depth_path: str, vadas_model: VADASFisheyeCameraModel, out_dir: str, file_identifier: str = None) -> np.ndarray:
    """
    이미 VADAS 좌표계로 된 깊이 맵을 직접 시각화 (일반 깊이 + 역깊이)
    analyze_distribution.py에서 생성된 matched_depth_*.png 파일 처리
    """
    # 1. VADAS 깊이 맵 로드
    depth_map = load_image_any(vadas_depth_path)  # 이미 VADAS 좌표계
    H, W = depth_map.shape
    
    print(f"[INFO] Loading VADAS depth map: {W}x{H}")
    print(f"[INFO] Depth range: {depth_map.min():.3f} - {depth_map.max():.3f}")
    
    # analyze_distribution.py에서 저장된 형식 확인 (16비트 PNG, 256배 스케일)
    if depth_map.max() > 100:  # 256배 스케일된 것으로 추정
        depth_map = depth_map / 256.0
        print(f"[INFO] Detected 16-bit PNG format, rescaled by 1/256")
        print(f"[INFO] Rescaled depth range: {depth_map.min():.3f} - {depth_map.max():.3f}")
    
    # 2. 유효 픽셀 분석
    valid_mask = depth_map > 0
    hole_mask = depth_map == 0
    valid_pixels = np.sum(valid_mask)
    hole_pixels = np.sum(hole_mask)
    total_pixels = W * H
    coverage_ratio = valid_pixels / total_pixels
    hole_ratio = hole_pixels / total_pixels
    
    print(f"[INFO] Valid pixels: {valid_pixels}/{total_pixels} ({coverage_ratio:.3f})")
    print(f"[INFO] Hole pixels: {hole_pixels}/{total_pixels} ({hole_ratio:.3f})")
    
    if valid_pixels > 0:
        valid_depths = depth_map[valid_mask]
        print(f"[INFO] Valid depth statistics:")
        print(f"  Mean: {np.mean(valid_depths):.3f}m")
        print(f"  Median: {np.median(valid_depths):.3f}m")
        print(f"  Std: {np.std(valid_depths):.3f}m")
        print(f"  P95: {np.percentile(valid_depths, 95):.3f}m")
    
    # 3. Inverse 깊이 맵 계산
    inverse_depth_map = np.zeros_like(depth_map)
    valid_depth_mask = depth_map > 1e-6  # 매우 작은 깊이 값 제외
    inverse_depth_map[valid_depth_mask] = 1.0 / depth_map[valid_depth_mask]
    
    print(f"[INFO] Inverse depth statistics:")
    if np.any(valid_depth_mask):
        valid_inverse_depths = inverse_depth_map[valid_depth_mask]
        print(f"  Inverse depth range: {valid_inverse_depths.min():.6f} - {valid_inverse_depths.max():.6f} (1/m)")
        print(f"  Mean inverse: {np.mean(valid_inverse_depths):.6f} (1/m)")
    
    # 4. VADAS 모델 정보 출력
    print(f"[INFO] VADAS model parameters:")
    print(f"  Image size: {vadas_model.image_size}")
    print(f"  Principal point offset: ({vadas_model.ux:.3f}, {vadas_model.uy:.3f})")
    
    # 5. 일반 깊이 맵 시각화 (구멍은 검은색으로)
    vis_colored_depth = colorize_image(depth_map, hole_color="black")
    
    if file_identifier:
        depth_label = f"VADAS Depth Map - {file_identifier} (Valid={coverage_ratio:.3f}, Holes={hole_ratio:.3f})"
    else:
        depth_label = f"VADAS Depth Map (Direct) - Valid={coverage_ratio:.3f}, Holes={hole_ratio:.3f}"
    
    vis_depth_labeled = put_label(vis_colored_depth, depth_label, org=(10, 28))
    
    # 6. Inverse 깊이 맵 시각화 (구멍은 흰색으로)
    vis_colored_inverse = colorize_image(inverse_depth_map, hole_color="white")
    
    if file_identifier:
        inverse_label = f"VADAS Inverse Depth - {file_identifier} (Valid={coverage_ratio:.3f}, Holes={hole_ratio:.3f})"
    else:
        inverse_label = f"VADAS Inverse Depth (Direct) - Valid={coverage_ratio:.3f}, Holes={hole_ratio:.3f}"
    
    vis_inverse_labeled = put_label(vis_colored_inverse, inverse_label, org=(10, 28))
    
    # 7. 추가: 구멍 마스크 시각화 (구멍 분포 확인용)
    hole_visualization = np.zeros((H, W, 3), dtype=np.uint8)
    hole_visualization[valid_mask] = (0, 255, 0)  # 유효 영역: 초록색
    hole_visualization[hole_mask] = (0, 0, 255)   # 구멍 영역: 빨간색
    
    if file_identifier:
        hole_label = f"VADAS Hole Distribution - {file_identifier} (Green=Valid, Red=Holes)"
    else:
        hole_label = f"VADAS Hole Distribution (Green=Valid, Red=Holes)"
    
    vis_hole_labeled = put_label(hole_visualization, hole_label, org=(10, 28))
    
    # 8. 결과 저장
    if file_identifier:
        out_path_depth_png = os.path.join(out_dir, f"vadas_fisheye_depth_{file_identifier}.png")
        out_path_depth_npy = os.path.join(out_dir, f"vadas_fisheye_depth_{file_identifier}.npy")
        out_path_inverse_png = os.path.join(out_dir, f"vadas_fisheye_inverse_depth_{file_identifier}.png")
        out_path_inverse_npy = os.path.join(out_dir, f"vadas_fisheye_inverse_depth_{file_identifier}.npy")
        out_path_hole_png = os.path.join(out_dir, f"vadas_hole_distribution_{file_identifier}.png")
    else:
        out_path_depth_png = os.path.join(out_dir, "vadas_fisheye_depth_direct.png")
        out_path_depth_npy = os.path.join(out_dir, "vadas_fisheye_depth_direct.npy")
        out_path_inverse_png = os.path.join(out_dir, "vadas_fisheye_inverse_depth_direct.png")
        out_path_inverse_npy = os.path.join(out_dir, "vadas_fisheye_inverse_depth_direct.npy")
        out_path_hole_png = os.path.join(out_dir, "vadas_hole_distribution_direct.png")
    
    # 저장
    cv.imwrite(out_path_depth_png, vis_depth_labeled)
    print(f"[SAVE] VADAS fisheye depth (holes=black): {out_path_depth_png}")
    
    cv.imwrite(out_path_inverse_png, vis_inverse_labeled)
    print(f"[SAVE] VADAS fisheye inverse depth (holes=white): {out_path_inverse_png}")
    
    cv.imwrite(out_path_hole_png, vis_hole_labeled)
    print(f"[SAVE] VADAS hole distribution visualization: {out_path_hole_png}")
    
    np.save(out_path_depth_npy, depth_map)
    print(f"[SAVE] Raw VADAS depth map data: {out_path_depth_npy}")
    
    np.save(out_path_inverse_npy, inverse_depth_map)
    print(f"[SAVE] Raw VADAS inverse depth map data: {out_path_inverse_npy}")
    
    # 9. 3개 이미지 비교 시각화 (Normal + Inverse + Holes)
    if file_identifier:
        comparison_path = os.path.join(out_dir, f"vadas_depth_triple_comparison_{file_identifier}.png")
    else:
        comparison_path = os.path.join(out_dir, "vadas_depth_triple_comparison_direct.png")
    
    # 세 이미지를 나란히 배치 (수직으로 배치하여 너무 넓지 않게)
    comparison_img = np.vstack([
        np.hstack([vis_depth_labeled, vis_inverse_labeled]),
        np.hstack([vis_hole_labeled, np.zeros_like(vis_hole_labeled)])  # 빈 공간으로 채움
    ])
    
    # 전체 비교 이미지에 제목 추가
    if file_identifier:
        comparison_label = f"VADAS Complete Analysis - {file_identifier} | TopLeft: Normal (holes=black) | TopRight: Inverse (holes=white) | BottomLeft: Hole Distribution"
    else:
        comparison_label = f"VADAS Complete Analysis | TopLeft: Normal | TopRight: Inverse | BottomLeft: Holes"
    
    comparison_labeled = put_label(comparison_img, comparison_label, 
                                 org=(10, 60), 
                                 scale=0.7, color=(255, 255, 0))
    
    cv.imwrite(comparison_path, comparison_labeled)
    print(f"[SAVE] VADAS triple comparison visualization: {comparison_path}")
    
    return depth_map


def detect_depth_map_source(image_path: str) -> str:
    """
    깊이 맵의 소스를 감지하여 적절한 처리 방식 결정
    """
    path_str = str(image_path).lower()
    
    if "matched_depth_" in path_str:
        return "vadas_matched"  # analyze_distribution.py에서 생성된 매칭된 깊이 맵
    elif "affine_depth_" in path_str:
        return "vadas_affine"   # analyze_distribution.py에서 생성된 affine 깊이 맵  
    elif any(keyword in path_str for keyword in ["depth_anything", "da_", "single_image"]):
        return "depth_anything"  # Depth Anything 원본 출력
    elif path_str.endswith(('.png', '.jpg', '.jpeg')) and len(os.path.basename(path_str).split('.')[0]) == 10: # & -> and 수정
        return "vadas_gt"       # GT VADAS 깊이 맵 (10자리 파일명)
    else:
        return "unknown"


def run_pipeline_with_backward_mapping(args):
    """
    기존 파이프라인에 Backward mapping 추가
    """
    os.makedirs(args.out_dir, exist_ok=True)

    # 1) 소스 감지
    source_type = detect_depth_map_source(args.image_path)
    print(f"[INFO] Detected depth map source: {source_type}")
    
    # 2) VADAS 모델 설정
    W_out, H_out = args.dst_size
    vadas_intrinsic = DEFAULT_CALIB['a6']['intrinsic']
    vadas_fish_model = VADASFisheyeCameraModel(vadas_intrinsic, image_size=(W_out, H_out))
    
    # 3) 파일 식별자 추출
    filename = os.path.basename(args.image_path)
    if "matched_depth_" in filename:
        file_identifier = filename.split("matched_depth_")[1].split(".")[0]
    elif "affine_depth_" in filename:
        file_identifier = filename.split("affine_depth_")[1].split(".")[0]
    else:
        file_identifier = os.path.splitext(filename)[0]
    
    # 4) VADAS 좌표계 깊이 맵인 경우는 직접 시각화
    if source_type in ["vadas_matched", "vadas_affine", "vadas_gt"]:
        print(f"[INFO] Processing as VADAS coordinate system depth map (Direct visualization)")
        depth_map = visualize_vadas_depth_map_directly(
            args.image_path, 
            vadas_fish_model, 
            args.out_dir,
            file_identifier
        )
        
    # 5) Depth Anything 같은 pinhole 깊이 맵인 경우는 backward mapping 적용
    elif source_type == "depth_anything" or source_type == "unknown":
        print(f"[INFO] Processing as pinhole depth map → VADAS fisheye (Backward mapping)")
        
        # 소스 이미지 로드
        image_src = load_image_any(args.image_path)
        H_src, W_src = image_src.shape[:2]
        print(f"[INFO] Loaded source image: {W_src}x{H_src}")
        print(f"[INFO] Source depth range: {image_src.min():.3f} - {image_src.max():.3f}")
        
        # 소스 카메라 intrinsic 설정 (FOV 직접 지정)
        src_fov_deg = args.src_fov_deg if args.src_fov_deg is not None else 90.0 # 기본값 90도
        K_src = K_from_fov(W_src, H_src, src_fov_deg)
        print(f"[INFO] Using source camera intrinsic with FOV: {src_fov_deg:.1f}°")
        
        # 회전 정렬
        R_fe2src = fix_rotation_alignment(vadas_fish_model, (W_out, H_out))
        
        # Backward mapping 수행
        fisheye_depth_map = backward_map_depth_to_fisheye(
            image_src_depth=image_src,
            K_src=K_src,
            vadas_fish_model=vadas_fish_model,
            R_fe2src=R_fe2src,
            dst_size=(W_out, H_out)
        )
        
        # 결과 시각화 및 저장
        vis_colored_depth = colorize_image(fisheye_depth_map, hole_color="black")
        estimated_fov = np.degrees(2*math.atan(W_src/(2*K_src[0,0])))
        coverage_ratio = np.sum(fisheye_depth_map > 0) / fisheye_depth_map.size
        
        label = f"Backward Mapped Fisheye Depth - {file_identifier} (Est.FOV={estimated_fov:.1f}°, Coverage={coverage_ratio:.3f})"
        vis_labeled = put_label(vis_colored_depth, label, org=(10, 28))

        # 결과 이미지 저장
        out_path_png = os.path.join(args.out_dir, f"backward_fisheye_depth_{file_identifier}.png")
        cv.imwrite(out_path_png, vis_labeled)
        print(f"[SAVE] Backward mapped fisheye depth visualization: {out_path_png}")

        # 원본 깊이 맵 데이터 저장 (NPY)
        out_path_npy = os.path.join(args.out_dir, f"backward_fisheye_depth_{file_identifier}.npy")
        np.save(out_path_npy, fisheye_depth_map)
        print(f"[SAVE] Raw backward mapped depth data: {out_path_npy}")

        # Inverse depth도 계산 및 저장
        inverse_depth_map = np.zeros_like(fisheye_depth_map)
        valid_mask = fisheye_depth_map > 1e-6
        inverse_depth_map[valid_mask] = 1.0 / fisheye_depth_map[valid_mask]
        
        vis_inverse = colorize_image(inverse_depth_map, hole_color="white")
        inverse_label = f"Backward Mapped Inverse Depth - {file_identifier} (Coverage={coverage_ratio:.3f})"
        vis_inverse_labeled = put_label(vis_inverse, inverse_label, org=(10, 28))
        
        out_path_inverse_png = os.path.join(args.out_dir, f"backward_fisheye_inverse_depth_{file_identifier}.png")
        cv.imwrite(out_path_inverse_png, vis_inverse_labeled)
        print(f"[SAVE] Backward mapped inverse depth visualization: {out_path_inverse_png}")
        
        np.save(os.path.join(args.out_dir, f"backward_fisheye_inverse_depth_{file_identifier}.npy"), inverse_depth_map)
        
        # 비교 시각화
        comparison_img = np.hstack([vis_labeled, vis_inverse_labeled])
        comparison_path = os.path.join(args.out_dir, f"backward_comparison_{file_identifier}.png")
        comparison_label = f"Backward Mapping Results - {file_identifier}: Normal (Left) vs Inverse (Right)"
        comparison_labeled = put_label(comparison_img, comparison_label, 
                                     org=(comparison_img.shape[1]//2 - 400, 60), 
                                     scale=0.8, color=(255, 255, 0))
        cv.imwrite(comparison_path, comparison_labeled)
        print(f"[SAVE] Backward mapping comparison: {comparison_path}")





# 기존 run_pipeline 함수를 backward mapping 버전으로 교체
def run_pipeline(args):
    """기존 파이프라인 함수를 스마트 버전으로 리디렉션"""
    return run_pipeline_with_backward_mapping(args)


def build_argparser():
    ap = argparse.ArgumentParser(description="VADAS Fisheye Depth Map Visualization")
    ap.add_argument("--image_path", type=str, 
                    default=r"C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\output\fisheye_da_931_results\output.overlay.png", 
                    help="Path to depth map (VADAS coordinate system expected).")
    ap.add_argument("--dst_size", type=str, default="1920x1536", help="Output size (should match input for VADAS maps).")
    ap.add_argument("--out_dir", type=str, default="./output/vadas_fisheye_visualization", help="Output directory.")

    args = ap.parse_args([]) if "__file__" not in globals() else ap.parse_args()

    # parse sizes
    W_out, H_out = parse_size(args.dst_size)
    args.dst_size = (W_out, H_out)
    return args


if __name__ == "__main__":
    args = build_argparser()
    run_pipeline(args)