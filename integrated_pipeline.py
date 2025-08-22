import numpy as np
import cv2 as cv
import math
import sys
import os
import shutil
from typing import List, Tuple, Optional, Any
from PIL import Image

# Dummy imports for Depth Anything V2 and VADAS model (will be replaced or properly integrated)
try:
    import torch
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation
    DEPTH_ANYTHING_AVAILABLE = True
except ImportError:
    print("Warning: torch or transformers not found. Depth Anything V2 functionality will be disabled.", file=sys.stderr)
    DEPTH_ANYTHING_AVAILABLE = False

# --- Constants from ref/ref_calibration_data.py ---
DEFAULT_CALIB = {
  "a6": {
    "model": "vadas",
    "intrinsic": [-0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391, #u2d
                  1.0447, 0.0021, 44.9516, 2.48822, # focal, pixel_size, cx, cy
                  0, 0.9965, -0.0067, -0.0956, 0.1006, -0.054, 0.0106], #d2u
    "extrinsic": [ 0.293769, -0.0542026, -0.631615, -0.00394431, -0.33116, -0.00963617 ],
    "image_size": None
  }
}

# --- VADASFisheyeCameraModel from ref/ref_camera_lidar_projector.py ---
class CameraModelBase:
    """Base class for camera projection models."""
    def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
        raise NotImplementedError

class VADASFisheyeCameraModel(CameraModelBase):
    """VADAS Polynomial Fisheye Camera Model, assuming +X is forward."""
    def __init__(self, intrinsic: List[float], image_size: Optional[Tuple[int, int]] = None):
        if len(intrinsic) < 18: # Changed from 11 to 18 to include d2u parameters
            raise ValueError("VADAS intrinsic must have at least 18 parameters (u2d + d2u).")
        self.k = intrinsic[0:7] # u2d parameters
        self.s = intrinsic[7] # focal
        self.div = intrinsic[8] # pixel_size
        self.ux = intrinsic[9] # cx
        self.uy = intrinsic[10] # cy
        self.d2u_k = intrinsic[11:18] # d2u parameters
        self.image_size = image_size

    def _poly_eval(self, coeffs: List[float], x: float) -> float:
        res = 0.0
        # Using Horner's method for polynomial evaluation, matching C++ implementation
        for c in reversed(coeffs):
            res = res * x + c
        return res

    def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
        # This model expects camera looking along +X axis.
        # The C++ code uses: normPt = cv::Point2f(-extrinsic_result(1), -extrinsic_result(2));
        # This corresponds to nx = -Yc, ny = -Zc
        nx = -Yc
        ny = -Zc
        
        dist = math.hypot(nx, ny)
        
        # C++: dist = dist < DBL_EPSILON ? DBL_EPSILON : dist;
        # This prevents division by zero. sys.float_info.epsilon is the Python equivalent of DBL_EPSILON.
        if dist < sys.float_info.epsilon:
            dist = sys.float_info.epsilon
        
        cosPhi = nx / dist
        sinPhi = ny / dist
        
        # C++: theta = atan2(dist, extrinsic_result(0));
        # This corresponds to theta = atan2(dist, Xc)
        theta = math.atan2(dist, Xc)

        # if Xc < 0: # Point is behind the camera
        #     return 0, 0, False

        xd = theta * self.s

        if abs(self.div) < 1e-9:
            return 0, 0, False
        
        # C++ polynomial evaluation loop is equivalent to this
        rd = self._poly_eval(self.k, xd) / self.div

        if math.isinf(rd) or math.isnan(rd):
            return 0, 0, False

        img_w_half = (self.image_size[0] / 2) if self.image_size else 0
        img_h_half = (self.image_size[1] / 2) if self.image_size else 0

        u = rd * cosPhi + self.ux + img_w_half
        v = rd * sinPhi + self.uy + img_h_half
        
        return int(round(u)), int(round(v)), True

# --- Visualization Utilities from fisheye_backward_mapping_image_sweep.py ---
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
            # 기본값: 검정색
            colored_img[hole_mask] = hole_colors["black"]
        
        return colored_img
    else:  # 컬러 이미지 (BGR)
        return img.astype(np.uint8)
    

def put_label(img: np.ndarray, text: str,
              org=(10, 28), font=cv.FONT_HERSHEY_SIMPLEX, scale=0.7,
              color=(255, 255, 255), thickness=2, lineType=cv.LINE_AA) -> np.ndarray:
    out = img.copy()
    cv.putText(out, text, org, font, scale, color, thickness, lineType)
    return out

# --- VADAS Undistortion Utilities from ref/vadas_undistortion_utils.py ---
def get_vadas_undistortion_maps(
    vadas_model_instance: VADASFisheyeCameraModel,
    original_image_size: Tuple[int, int],  # (width, height)
    rectified_size: Tuple[int, int],       # (width, height)
    rectified_K: Optional[np.ndarray] = None,
    depth_plane: float = 1.0
) -> Tuple[np.ndarray, np.ndarray]:
    """
    BACKWARD (sampling) mapping version (simplified & vectorized).

    - rectified_size: '스케치북' 캔버스 크기
    - rectified_K 없으면 fx=fy=s/div, cx,cy=캔버스 중앙
    - map_x/map_y는 rectified(출력) → 원본 fisheye 좌표. 범위 밖은 -1 유지 -> cv.remap 가 검정(0)으로 채움.
    """
    W_orig, H_orig = original_image_size
    W_rect, H_rect = rectified_size

    if rectified_K is None:
        # 픽셀 초점거리(px) = s / div (해상도와 독립, 캔버스만 키우면 자연스러운 '줌아웃' 효과)
        s = float(vadas_model_instance.s)
        div = float(vadas_model_instance.div)
        if abs(div) < 1e-12:
            raise ValueError("Invalid pixel_size (div) in VADAS intrinsic.")
        fx = fy = s / div
        cx = W_rect / 2.0
        cy = H_rect / 2.0
        rectified_K = np.array([[fx, 0, cx],
                                [0, fy, cy],
                                [0,  0,  1]], dtype=np.float64)
    else:
        fx, fy = rectified_K[0, 0], rectified_K[1, 1]
        cx, cy = rectified_K[0, 2], rectified_K[1, 2]

    # 출력(캔버스) 픽셀 그리드
    u = np.arange(W_rect, dtype=np.float32)
    v = np.arange(H_rect, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)

    # 가상 pinhole 정규화 평면
    x_std = (uu - cx) / fx
    y_std = (vv - cy) / fy
    z_std = np.ones_like(x_std)

    # 표준 → VADAS 좌표 (Xf,Yr,Zd)=(z,x,y)
    Xf = z_std
    Yr = x_std
    Zd = y_std

    # project_point 부호 일치 (Xf,-Yr,-Zd)
    Yc = -Yr
    Zc = -Zd
    Xc = Xf
    valid = Xc > 0  # 카메라 뒤 제외

    nx = -Yc  # = Yr
    ny = -Zc  # = Zd
    dist = np.hypot(nx, ny)
    eps = np.finfo(np.float32).eps
    dist_safe = np.where(dist < eps, eps, dist)
    cosPhi = nx / dist_safe
    sinPhi = ny / dist_safe
    theta = np.arctan2(dist, Xc)

    # u2d 다항식
    k = vadas_model_instance.k
    s_poly = vadas_model_instance.s
    div = vadas_model_instance.div
    ux = vadas_model_instance.ux
    uy = vadas_model_instance.uy

    xd = theta * s_poly
    rd = np.zeros_like(xd, dtype=np.float64)
    for c in reversed(k):
        rd = rd * xd + c
    if abs(div) < 1e-12:
        div = 1.0
    rd /= div

    invalid_rd = ~np.isfinite(rd)
    valid &= ~invalid_rd

    # 원본 이미지 중심 기준 오프셋
    img_w_half = W_orig / 2.0
    img_h_half = H_orig / 2.0
    u_fish = rd * cosPhi + ux + img_w_half
    v_fish = rd * sinPhi + uy + img_h_half

    # 원본 범위 내만 유효
    in_bounds = (u_fish >= 0) & (u_fish <= (W_orig - 1)) & (v_fish >= 0) & (v_fish <= (H_orig - 1))
    final_valid = valid & in_bounds

    # remap lookup (-1=invalid)
    map_x = np.full((H_rect, W_rect), -1, dtype=np.float32)
    map_y = np.full((H_rect, W_rect), -1, dtype=np.float32)
    map_x[final_valid] = u_fish[final_valid].astype(np.float32)
    map_y[final_valid] = v_fish[final_valid].astype(np.float32)
    return map_x, map_y

# --- Core Pipeline Functions (Implemented) ---

def _compute_source_coverage(map_x: np.ndarray, map_y: np.ndarray, src_size: Tuple[int, int]) -> Tuple[float, np.ndarray]:
    """
    원본(fisheye) 이미지 픽셀 커버리지 계산.
    - map_x/map_y가 가리키는 원본 픽셀 좌표를 라운딩해 커버리지 맵을 만든 뒤,
      원본 전체 픽셀 중 적어도 한 번 샘플된 비율을 반환.
    """
    W_src, H_src = src_size
    cov = np.zeros((H_src, W_src), dtype=np.uint8)
    valid = (map_x >= 0) & (map_y >= 0)
    if not np.any(valid):
        return 0.0, cov
    u = np.clip(np.rint(map_x[valid]).astype(np.int32), 0, W_src - 1)
    v = np.clip(np.rint(map_y[valid]).astype(np.int32), 0, H_src - 1)
    cov[v, u] = 1
    return float(cov.mean()), cov

def distort_to_undistort(image: np.ndarray,
                         vadas_model: VADASFisheyeCameraModel,
                         rectified_size: Optional[Tuple[int, int]] = None,
                         debug_mode: bool = True,
                         zoom: float = 1.0,
                         auto_cover_all: bool = True,
                         target_source_coverage: float = 0.995,
                         max_iters: int = 10,
                         zoom_step: float = 1.15) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    LDC by pure backward mapping on a large canvas (스케치북).
    - 중심을 캔버스 중앙에 고정.
    - 캔버스를 '채우지 않음': 범위 밖은 map=-1 → 검정.
    - zoom > 1.0 이면 줌아웃(fx,fy 감소). auto_cover_all=True 이면 원본 fisheye 커버리지가
      target_source_coverage 에 도달할 때까지 fx,fy 자동 감소(줌아웃) 시도.
    - rectified_size 를 크게 줄수록(예: 2x) 더 많은 영역을 담을 수 있음.
    """
    H_orig, W_orig = image.shape[:2]
    if rectified_size is None:
        W_rect, H_rect = W_orig, H_orig
    else:
        W_rect, H_rect = rectified_size

    # 기본 focal(px)
    s = float(vadas_model.s)
    div = float(vadas_model.div)
    if abs(div) < 1e-12:
        raise ValueError("Invalid pixel_size (div) in VADAS intrinsic.")
    f_base = s / div

    def build_maps(curr_zoom: float) -> Tuple[np.ndarray, np.ndarray, float, float, float, float]:
        fx = max(f_base / max(curr_zoom, 1e-6), 1e-6)
        fy = fx
        cx = W_rect / 2.0
        cy = H_rect / 2.0
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]], dtype=np.float64)
        mx, my = get_vadas_undistortion_maps(
            vadas_model_instance=vadas_model,
            original_image_size=(W_orig, H_orig),
            rectified_size=(W_rect, H_rect),
            rectified_K=K
        )
        return mx, my, fx, fy, cx, cy

    # 초기 맵 생성
    map_x, map_y, fx, fy, cx, cy = build_maps(zoom)

    # 원본(fisheye) 픽셀 커버리지 최대화 (옵션)
    if auto_cover_all:
        src_cov_ratio, _ = _compute_source_coverage(map_x, map_y, (W_orig, H_orig))
        it = 0
        while (src_cov_ratio < target_source_coverage) and (it < max_iters):
            zoom *= zoom_step  # 더 줌아웃
            map_x, map_y, fx, fy, cx, cy = build_maps(zoom)
            src_cov_ratio, _ = _compute_source_coverage(map_x, map_y, (W_orig, H_orig))
            it += 1
            if debug_mode:
                rect_valid = ((map_x >= 0) & (map_y >= 0)).mean()
                print(f"[AUTO-ZOOM] it={it} zoom={zoom:.3f} src_coverage={src_cov_ratio:.4f} rect_valid={rect_valid:.4f}")

    # 최종 remap (범위 밖은 검정)
    undistorted = cv.remap(
        image, map_x, map_y,
        interpolation=cv.INTER_LINEAR,
        borderMode=cv.BORDER_CONSTANT,
        borderValue=(0, 0, 0)
    )

    if debug_mode:
        valid_mask = ((map_x >= 0) & (map_y >= 0)).astype(np.uint8)
        src_cov_ratio, _ = _compute_source_coverage(map_x, map_y, (W_orig, H_orig))
        print(f"[LDC] rectified={W_rect}x{H_rect} zoom={zoom:.3f} rect_valid_ratio={valid_mask.mean():.3f} src_coverage={src_cov_ratio:.3f}")
        return undistorted, valid_mask

    return undistorted, None

def undistort_to_distort(image: np.ndarray, vadas_model: VADASFisheyeCameraModel, debug_mode: bool = False) -> np.ndarray:
    """
    Undistorted Image -> Distorted Image (VADAS Model)
    """
    print("[STEP] Undistorted -> Distorted conversion (Placeholder)")
    # Placeholder: This would use backward mapping from fisheye_backward_mapping_image_sweep.py
    # For now, return a dummy distorted image.
    H, W, _ = image.shape
    distorted_image = cv.resize(image, (W, H), interpolation=cv.INTER_LINEAR) # Just a resized copy for now
    return distorted_image

def run_depth_anything_v2(undistorted_image: np.ndarray, depth_anything_model: Any) -> np.ndarray:
    """
    Runs Depth Anything v2 on the undistorted image.
    Returns: depth_map (normalized 0-1)
    """
    print("[STEP] Running Depth Anything V2 (Placeholder)")
    if not DEPTH_ANYTHING_AVAILABLE:
        print("  - Depth Anything V2 is not available. Skipping.")
        # Return a dummy depth map (e.g., gradient or solid color)
        H, W, _ = undistorted_image.shape
        dummy_depth = np.linspace(0, 1, W, dtype=np.float32)[None, :].repeat(H, axis=0)
        return dummy_depth
    
    # Placeholder for actual Depth Anything inference
    # For now, assume depth_anything_model is a dummy or None
    # In a real scenario, this would involve:
    # rgb_pil = Image.fromarray(cv.cvtColor(undistorted_image, cv.COLOR_BGR2RGB))
    # depth01, raw_depth_tensor = depth_anything_model.run_inference(rgb_pil)
    # return depth01
    
    H, W, _ = undistorted_image.shape
    dummy_depth = np.linspace(0, 1, W, dtype=np.float32)[None, :].repeat(H, axis=0)
    return dummy_depth

def crop_in_normalized_plane(normalized_data: np.ndarray, crop_region: Tuple[float, float, float, float]) -> np.ndarray:
    """
    Crops normalized_data by normalized region (x_min,y_min,x_max,y_max in [0,1]).
    - Supports 2D (H,W) and 3D (H,W,C).
    - Returns a copy of the cropped array.
    """
    if normalized_data.ndim == 2:
        H, W = normalized_data.shape
        is_3d = False
    elif normalized_data.ndim == 3:
        H, W, _ = normalized_data.shape
        is_3d = True
    else:
        raise ValueError(f"Unsupported normalized_data.ndim={normalized_data.ndim} (expected 2 or 3)")

    x0n, y0n, x1n, y1n = map(float, crop_region)
    # Clamp to [0,1]
    x0n = float(np.clip(x0n, 0.0, 1.0))
    y0n = float(np.clip(y0n, 0.0, 1.0))
    x1n = float(np.clip(x1n, 0.0, 1.0))
    y1n = float(np.clip(y1n, 0.0, 1.0))

    # Ensure proper ordering and non-zero size
    if x1n <= x0n:
        x1n = min(1.0, x0n + (1.0 / max(1, W)))
    if y1n <= y0n:
        y1n = min(1.0, y0n + (1.0 / max(1, H)))

    # Convert to pixel indices (end-exclusive)
    x0 = int(np.floor(x0n * W))
    y0 = int(np.floor(y0n * H))
    x1 = int(np.ceil(x1n * W))
    y1 = int(np.ceil(y1n * H))

    # Clamp indices and enforce at least 1 pixel
    x0 = max(0, min(x0, W - 1))
    y0 = max(0, min(y0, H - 1))
    x1 = max(x0 + 1, min(x1, W))
    y1 = max(y0 + 1, min(y1, H))

    if is_3d:
        return normalized_data[y0:y1, x0:x1, :].copy()
    else:
        return normalized_data[y0:y1, x0:x1].copy()

def debug_visualization(step_name: str, image: np.ndarray, save_intermediate: bool = False, output_dir: str = "./output/debug_viz") -> None:
    """
    Visualizes an image for debugging purposes.
    Displays in a window and optionally saves to disk.
    """
    print(f"[DEBUG_VIZ] Displaying: {step_name}")
    
    # Ensure image is 3-channel for display, using colorize_image for 2D inputs
    if image.ndim == 2:
        display_img = colorize_image(image)
    else:
        display_img = image.astype(np.uint8)

    # Add label to the image
    labeled_img = put_label(display_img, step_name)

    if save_intermediate:
        # Ensure the output directory exists
        os.makedirs(output_dir, exist_ok=True)
        save_path = os.path.join(output_dir, f"{step_name.replace(' ', '_').replace(':', '')}.png")
        cv.imwrite(save_path, labeled_img)
        print(f"  - Saved intermediate image to: {save_path}")

def precompute_ldc_preview_cache(image: np.ndarray,
                                 vadas_model: VADASFisheyeCameraModel,
                                 rectified_size: Tuple[int, int],
                                 zoom_list: List[float],
                                 downscale: int = 2) -> List[Tuple[float, np.ndarray, float, float]]:
    """
    Precompute low-res LDC previews for smoother slider experience.
    - downscale: linear downscale of the canvas size (2 -> 1/4 pixels).
    Returns list of tuples: (zoom, preview_bgr, rect_valid_ratio, src_coverage)
    """
    H_orig, W_orig = image.shape[:2]
    W_rect, H_rect = rectified_size
    Wp, Hp = max(1, W_rect // downscale), max(1, H_rect // downscale)

    s = float(vadas_model.s)
    div = float(vadas_model.div)
    if abs(div) < 1e-12:
        raise ValueError("Invalid pixel_size (div) in VADAS intrinsic.")
    f_base = s / div

    previews: List[Tuple[float, np.ndarray, float, float]] = []
    print(f"[CACHE] Precomputing {len(zoom_list)} previews at {Wp}x{Hp} (downscale={downscale})...")
    for i, z in enumerate(zoom_list, 1):
        fx = max(f_base / max(z, 1e-6), 1e-6)
        fy = fx
        cx = Wp / 2.0
        cy = Hp / 2.0
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]], dtype=np.float64)

        mx, my = get_vadas_undistortion_maps(
            vadas_model_instance=vadas_model,
            original_image_size=(W_orig, H_orig),
            rectified_size=(Wp, Hp),
            rectified_K=K
        )
        preview = cv.remap(
            image, mx, my,
            interpolation=cv.INTER_LINEAR,
            borderMode=cv.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )
        rect_valid_ratio = float(((mx >= 0) & (my >= 0)).mean())
        src_cov_ratio, _ = _compute_source_coverage(mx, my, (W_orig, H_orig))

        labeled = put_label(preview, f"zoom={z:.2f}  rect_valid={rect_valid_ratio:.3f}  src_cov={src_cov_ratio:.3f}",
                            org=(10, 28), scale=0.6, color=(255, 255, 255))
        previews.append((z, labeled, rect_valid_ratio, src_cov_ratio))
        if i % max(1, len(zoom_list)//10) == 0 or i == len(zoom_list):
            print(f"  - {i}/{len(zoom_list)} cached")

    print("[CACHE] Done.")
    return previews

def run_ldc_zoom_viewer(image: np.ndarray,
                        vadas_model: VADASFisheyeCameraModel,
                        rectified_size: Optional[Tuple[int, int]] = None,
                        init_zoom: float = 1.0,
                        win_name: str = "LDC Zoom Viewer (q: quit, s: save)",
                        use_cache: bool = True,
                        cache_downscale: int = 2,
                        min_zoom: float = 0.25,
                        max_zoom: float = 4.0,
                        cache_steps: int = 80) -> None:
    """
    Interactive viewer for LDC on a large canvas using backward mapping.
    - Trackbar 'zoom x100': >100 -> zoom-out (wider FOV), <100 -> zoom-in.
    - Center is fixed at canvas center. We do not force-fill; out-of-bounds stay black.
    - use_cache=True: precompute low-res previews for smooth sliding.
    """
    H_orig, W_orig = image.shape[:2]
    if rectified_size is None:
        # 기본: 2배 스케치북
        W_rect, H_rect = W_orig * 2, H_orig * 2
    else:
        W_rect, H_rect = rectified_size

    # 픽셀 초점 (기본): s/div
    s = float(vadas_model.s)
    div = float(vadas_model.div)
    if abs(div) < 1e-12:
        raise ValueError("Invalid pixel_size (div) in VADAS intrinsic.")
    f_base = s / div

    # 윈도우 & 트랙바
    cv.namedWindow(win_name, cv.WINDOW_NORMAL)
    # 캐시 해상도 기준으로 창 크기 설정(작게 빠르게)
    disp_w, disp_h = (W_rect // cache_downscale) if use_cache else W_rect, (H_rect // cache_downscale) if use_cache else H_rect
    cv.resizeWindow(win_name, min(disp_w, 1920), min(disp_h, 1080))

    # Trackbar 범위 설정
    tb_min = int(round(min_zoom * 100))
    tb_max = int(round(max_zoom * 100))
    tb_init = int(np.clip(round(init_zoom * 100), tb_min, tb_max))
    cv.createTrackbar("zoom x100", win_name, tb_init, tb_max, lambda v: None)
    # OpenCV 트랙바는 하한이 0이므로, 갱신 때 tb_min 반영

    # 캐시 준비
    cache: List[Tuple[float, np.ndarray, float, float]] = []
    if use_cache:
        zoom_list = list(np.linspace(min_zoom, max_zoom, cache_steps))
        cache = precompute_ldc_preview_cache(image, vadas_model, (W_rect, H_rect), zoom_list, downscale=cache_downscale)
        cache_zooms = np.array([z for z, _, _, _ in cache], dtype=np.float32)

    # 헬퍼: 맵 생성 (비캐시)
    def build_maps(curr_zoom: float) -> Tuple[np.ndarray, np.ndarray]:
        fx = max(f_base / max(curr_zoom, 1e-6), 1e-6)  # 줌아웃시 focal 감소
        fy = fx
        cx = W_rect / 2.0
        cy = H_rect / 2.0
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]], dtype=np.float64)
        mx, my = get_vadas_undistortion_maps(
            vadas_model_instance=vadas_model,
            original_image_size=(W_orig, H_orig),
            rectified_size=(W_rect, H_rect),
            rectified_K=K
        )
        return mx, my

    last_zoom = -1.0
    last_vis = None
    while True:
        raw = cv.getTrackbarPos("zoom x100", win_name)
        raw = max(tb_min, raw)  # 하한 적용
        zoom = raw / 100.0

        if abs(zoom - last_zoom) > 1e-3 or last_vis is None:
            if use_cache and cache:
                # 캐시에서 가장 가까운 줌 찾기
                idx = int(np.argmin(np.abs(cache_zooms - zoom)))
                _, preview_bgr, _, _ = cache[idx]
                last_vis = preview_bgr
            else:
                # 비캐시: 원본 캔버스로 즉시 계산
                map_x, map_y = build_maps(zoom)
                undistorted = cv.remap(
                    image, map_x, map_y,
                    interpolation=cv.INTER_LINEAR,
                    borderMode=cv.BORDER_CONSTANT,
                    borderValue=(0, 0, 0)
                )
                valid_mask = ((map_x >= 0) & (map_y >= 0)).astype(np.uint8)
                rect_valid_ratio = float(valid_mask.mean())
                src_cov_ratio, _ = _compute_source_coverage(map_x, map_y, (W_orig, H_orig))
                last_vis = put_label(undistorted, f"zoom={zoom:.2f}  rect_valid={rect_valid_ratio:.3f}  src_cov={src_cov_ratio:.3f}",
                                     org=(10, 28), scale=0.7, color=(255, 255, 255))
            last_zoom = zoom

        cv.imshow(win_name, last_vis)
        key = cv.waitKey(15) & 0xFF  # 짧은 대기(부드러운 슬라이드)
        if key in (27, ord('q')):  # ESC or q
            break
        if key == ord('s'):
            os.makedirs("./output/debug_viz", exist_ok=True)
            out_path = f"./output/debug_viz/undistorted_zoom_{int(round(zoom*100))}.png"
            cv.imwrite(out_path, last_vis)
            print(f"[SAVE] {out_path}")

    cv.destroyWindow(win_name)

# --- Main Pipeline Execution Example ---
def integrated_pipeline_example(debug_interactive: bool = True):
    print("--- Starting Depth Anything + VADAS Pipeline Example ---")

    # Load input image
    image_path = "C:\\Users\\seok436\\Documents\\VSCode\\Projects\\point-cloud-creation\\point-cloud-creation\\ncdb-cls-sample\\synced_data\\image_a6\\0000000931.jpg"
    input_image = cv.imread(image_path)
    if input_image is None:
        print(f"Error: Could not load image from {image_path}", file=sys.stderr)
        return
    debug_visualization("1. Fisheye Input Image", input_image, save_intermediate=True, output_dir="./output/debug_viz/")

    # Initialize VADAS model with original image size
    print(f"Length of intrinsic: {len(DEFAULT_CALIB['a6']['intrinsic'])}")
    vadas_model = VADASFisheyeCameraModel(DEFAULT_CALIB['a6']['intrinsic'], image_size=(input_image.shape[1], input_image.shape[0]))

    # Step 1: Distorted -> Undistorted on a larger 'sketchbook' canvas
    H, W = input_image.shape[:2]
    scale = 2.0  # 스케치북 배율 (가득 채우지 않고, 중앙 정렬, 가능한 많은 어안 영역 표시)
    W_rect, H_rect = int(W * scale), int(H * scale)

    if debug_interactive:
        # [Interactive Debug] Zoom slider viewer (press q to exit)
        run_ldc_zoom_viewer(
            input_image,
            vadas_model,
            rectified_size=(W_rect, H_rect),
            init_zoom=1.0,
            win_name="LDC Zoom Viewer (q: quit, s: save)",
            use_cache=True,
            cache_downscale=2,     # 한 이미지 사이즈가 4배(=픽셀 수 1/4) 작게
            min_zoom=0.25,
            max_zoom=4.0,
            cache_steps=100        # 캐시 샘플 수(부드러움 조절)
        )
        # 인터랙티브 이후 기본 결과도 저장
        undistorted_image, valid_mask = distort_to_undistort(
            input_image,
            vadas_model,
            rectified_size=(W_rect, H_rect),  # 큰 캔버스
            debug_mode=True,
            zoom=1.0,
            auto_cover_all=False
        )
        debug_visualization("2. Undistorted (Large Canvas, Centered)", undistorted_image, save_intermediate=True, output_dir="./output/debug_viz/")
    else:
        # Debug False: 기본 값으로 넓은 스케치북에 중앙점에 맞춘 결과를 저장
        undistorted_image, _ = distort_to_undistort(
            input_image,
            vadas_model,
            rectified_size=(W_rect, H_rect),  # 큰 캔버스
            debug_mode=False,
            zoom=1.0,
            auto_cover_all=True  # 가능한 많은 포인트를 포함 시도
        )
        os.makedirs("./output/debug_viz", exist_ok=True)
        out_path = "./output/debug_viz/undistorted_large_canvas.png"
        cv.imwrite(out_path, undistorted_image)
        print(f"[SAVE] Undistorted (large canvas, centered) -> {out_path}")

    # Step 2: Run Depth Anything v2
    depth_anything_model = None 
    depth_map = run_depth_anything_v2(undistorted_image, depth_anything_model)
    debug_visualization("3. Depth Map (from DA v2)", depth_map, save_intermediate=True, output_dir="./output/debug_viz/")

    # Step 3: Undistorted Depth -> Distorted Depth
    distorted_depth_image = undistort_to_distort(undistorted_image, vadas_model)
    debug_visualization("4. Distorted Depth Image", distorted_depth_image, save_intermediate=True, output_dir="./output/debug_viz/")

    # Step 4: Crop in Normalized Plane (if normalized_plane_data was generated)
    # 현재는 valid_mask(2D)로 예시
    if debug_interactive:
        # 예시 crop
        crop_region = (0.15, 0.15, 0.85, 0.85)
        if 'valid_mask' in locals() and valid_mask is not None:
            cropped_normalized_data = crop_in_normalized_plane(valid_mask, crop_region)
            if cropped_normalized_data.ndim == 3:
                debug_visualization("5. Cropped Normalized Data (First Channel)", cropped_normalized_data[:,:,0], save_intermediate=True, output_dir="./output/debug_viz/")
            elif cropped_normalized_data.ndim == 2:
                debug_visualization("5. Cropped Normalized Data", cropped_normalized_data, save_intermediate=True, output_dir="./output/debug_viz/5_cropped_normalized_data")

    print("--- Pipeline Example Finished. ---")

if __name__ == "__main__":
    # 디버그 모드: True일 때만 인터랙티브 뷰어 동작
    integrated_pipeline_example(debug_interactive=True)
