"""
Standalone test script:
Pinhole depth (Depth Anything 등) -> VADAS fisheye depth 재투영 (점군 기반, z-buffer).

Assumptions:
- 동일한 중심 (no translation)
- 입력 depth 단위: meter (16bit PNG 는 /256 스케일)
- 회전: VADAS 중심 광선이 (0,0,1) 정면이 되도록 자동 정렬
"""

import os, math, argparse
import numpy as np
import cv2 as cv
from pathlib import Path
from typing import Tuple
from ref.ref_calibration_data import DEFAULT_CALIB
from ref.ref_camera_lidar_projector import VADASFisheyeCameraModel

# ---------------- Utilities ----------------
def load_depth(path: str) -> np.ndarray:
    ext = os.path.splitext(path)[1].lower()
    if ext == ".npy":
        d = np.load(path).astype(np.float32)
    else:
        img = cv.imread(path, cv.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(path)
        d = img.astype(np.float32)
        if d.dtype == np.uint16 or d.max() > 100:
            d = d / 256.0
    if d.ndim != 2:
        raise ValueError("Depth must be single channel")
    return d

def k_from_fov(w: int, h: int, fov_deg: float) -> np.ndarray:
    fov_deg = min(fov_deg, 179.9)
    theta = math.radians(fov_deg)
    fx = w / (2.0 * math.tan(theta / 2.0))
    fy = fx
    cx, cy = w/2.0, h/2.0
    return np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=np.float64)

def compute_rotation_between_vectors(v1, v2):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    c = np.cross(v1, v2)
    d = np.dot(v1, v2)
    if np.linalg.norm(c) < 1e-12:
        return np.eye(3) if d > 0 else -np.eye(3)
    K = np.array([[0, -c[2], c[1]],
                  [c[2], 0, -c[0]],
                  [-c[1], c[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - d) / (np.linalg.norm(c)**2))

def vadas_center_ray(vadas_model: VADASFisheyeCameraModel) -> np.ndarray:
    # 중앙 픽셀 (정확한 principal offset 반영)
    W, H = vadas_model.image_size
    u = W/2.0
    v = H/2.0
    # 역투영 근사: rd=0 => ray = [1,0,0] in VADAS (X_forward,Y_right,Z_down)
    return np.array([1.0, 0.0, 0.0], dtype=np.float64)

def colorize(depth: np.ndarray, hole_color=(0,0,0)) -> np.ndarray:
    if depth.size == 0:
        return np.zeros((1,1,3), np.uint8)
    valid = depth > 0
    out = np.zeros((*depth.shape,3), np.uint8)
    if np.any(valid):
        d = depth.copy()
        mn, mx = d[valid].min(), d[valid].max()
        if mx - mn < 1e-6:
            norm = np.zeros_like(d, dtype=np.uint8)
        else:
            norm = ((d - mn)/(mx-mn)*255).clip(0,255).astype(np.uint8)
        cm = cv.applyColorMap(norm, cv.COLORMAP_JET)
        out[valid] = cm[valid]
    out[~valid] = hole_color
    return out

# ---------------- Core Reprojection ----------------
def reproject_pinhole_depth_to_fisheye(
    depth_src: np.ndarray,
    K_src: np.ndarray,
    vadas_fish_model: VADASFisheyeCameraModel,
    R_fe2src: np.ndarray,
    dst_size: Tuple[int,int],
    store_mode: str = "range",
    splat_radius: int = 1,
    hole_fill: bool = True
) -> np.ndarray:
    """
    Reproject pinhole depth → VADAS fisheye depth.

    Axis conventions:
      Standard pinhole:  (x_right, y_down, z_forward)
      VADAS fisheye:     (X_forward, Y_right, Z_down)

    VADASFisheyeCameraModel.project_point internally does:
        nx = -Yc
        ny = -Zc
        theta = atan2( sqrt(nx^2+ny^2), Xc )

    그래서 우리가 (X_forward, Y_right, Z_down)을 그대로 (Xc,Yc,Zc)로 넣으면
    nx = -Y_right, ny = -Z_down 이 되어 수평/수직이 뒤집힘(특히 상하 반전).

    올바른 보정:
        원하는 평면 좌표 = ( +Y_right, +Z_down )
        모델 내부에서 nx = -Yc, ny = -Zc 이므로
            -Yc = +Y_right  ⇒ Yc = -Y_right
            -Zc = +Z_down   ⇒ Zc = -Z_down

    따라서 project_point 호출 시 (Xc, Yc, Zc) = (X_forward, -Y_right, -Z_down).
    """
    Hs, Ws = depth_src.shape
    Wf, Hf = dst_size
    fx, fy, cx, cy = K_src[0,0], K_src[1,1], K_src[0,2], K_src[1,2]

    u = np.arange(Ws)
    v = np.arange(Hs)
    uu, vv = np.meshgrid(u, v)
    d = depth_src
    valid = d > 0
    if not np.any(valid):
        return np.zeros((Hf, Wf), np.float32)

    uu = uu[valid].astype(np.float32)
    vv = vv[valid].astype(np.float32)
    dd = d[valid].astype(np.float32)

    # Unproject (standard: x_right, y_down, z_forward)
    X = (uu - cx)/fx * dd
    Y = (vv - cy)/fy * dd
    Z = dd
    P_src = np.stack([X, Y, Z], axis=1)

    # source -> fisheye (R_fe2src is fe->src so transpose)
    P_fe_std = (R_fe2src.T @ P_src.T).T

    # Standard -> VADAS mapping (without sign flips yet):
    #   Xf = z_forward
    #   Yr = x_right
    #   Zd = y_down
    Xf = P_fe_std[:, 2]
    Yr = P_fe_std[:, 0]
    Zd = P_fe_std[:, 1]

    if store_mode == "forward":
        depth_val = Xf
    else:
        depth_val = np.sqrt(Xf*Xf + Yr*Yr + Zd*Zd)

    N = depth_val.shape[0]
    u_fe = np.empty(N, np.float32)
    v_fe = np.empty(N, np.float32)
    ok_mask = np.zeros(N, bool)

    # --------- FIX: sign-correct Y_right & Z_down before projection ---------
    # Pass (Xc, Yc, Zc) = (Xf, -Yr, -Zd)
    for i in range(N):
        dv = depth_val[i]
        if dv <= 0:
            continue
        u_proj, v_proj, ok = vadas_fish_model.project_point(Xf[i], -Yr[i], -Zd[i])
        if ok and 0 <= u_proj < Wf and 0 <= v_proj < Hf:
            u_fe[i] = u_proj
            v_fe[i] = v_proj
            ok_mask[i] = True
    # ------------------------------------------------------------------------

    idx = np.where(ok_mask)[0]
    if idx.size == 0:
        return np.zeros((Hf, Wf), np.float32)

    depth_fe = np.zeros((Hf, Wf), np.float32)
    zbuf = np.full((Hf, Wf), np.inf, np.float32)

    offsets = [(0, 0)]
    if splat_radius > 0:
        offsets = []
        for dy in range(-splat_radius, splat_radius + 1):
            for dx in range(-splat_radius, splat_radius + 1):
                offsets.append((dx, dy))

    for i in idx:
        dv = depth_val[i]
        uf = u_fe[i]
        vf = v_fe[i]
        uc = int(round(uf))
        vc = int(round(vf))
        for dx, dy in offsets:
            x = uc + dx
            y = vc + dy
            if 0 <= x < Wf and 0 <= y < Hf:
                if dv < zbuf[y, x]:
                    zbuf[y, x] = dv
                    depth_fe[y, x] = dv

    if hole_fill:
        holes = depth_fe == 0
        if np.any(holes):
            kernel = np.ones((3, 3), np.uint8)
            dil = cv.dilate((depth_fe > 0).astype(np.uint8), kernel, 1)
            border = (dil == 1) & holes
            ys, xs = np.where(border)
            for y, x in zip(ys, xs):
                patch = depth_fe[max(0, y-1):min(Hf, y+2), max(0, x-1):min(Wf, x+2)]
                vals = patch[patch > 0]
                if vals.size > 0:
                    depth_fe[y, x] = vals.mean()

    return depth_fe

# ---------------- Pipeline ----------------
def run(args):
    depth_src = load_depth(args.image_path)
    Hs, Ws = depth_src.shape
    print(f"[INFO] Source depth loaded: {Ws}x{Hs}, min={depth_src[depth_src>0].min() if np.any(depth_src>0) else 0:.3f}, max={depth_src.max():.3f}")

    if args.fx is not None and args.fy is not None:
        fx, fy = args.fx, args.fy
        cx = args.cx if args.cx is not None else Ws/2
        cy = args.cy if args.cy is not None else Hs/2
        K_src = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=np.float64)
        print(f"[INFO] Using provided intrinsics fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    else:
        K_src = k_from_fov(Ws, Hs, args.src_fov_deg)
        print(f"[INFO] Using FOV={args.src_fov_deg:.1f}° -> fx={K_src[0,0]:.2f}")

    Wf, Hf = args.out_size
    vadas_intrinsic = DEFAULT_CALIB['a6']['intrinsic']
    vadas_model = VADASFisheyeCameraModel(vadas_intrinsic, image_size=(Wf, Hf))

    center_ray_vadas = vadas_center_ray(vadas_model)            # (1,0,0) in VADAS
    center_ray_std = np.array([0,0,1], dtype=np.float64)        # target forward in std
    # Convert VADAS center to std: VADAS(Xf,Yr,Zd) -> std(x,y,z) = (Yr, Zd, Xf)
    # For center (1,0,0) => std=(0,0,1)
    src_vec_std = np.array([center_ray_vadas[1], center_ray_vadas[2], center_ray_vadas[0]], dtype=np.float64)
    R_fe2src = compute_rotation_between_vectors(src_vec_std, center_ray_std)
    print(f"[INFO] R_fe2src:\n{R_fe2src}")

    depth_fe = reproject_pinhole_depth_to_fisheye(
        depth_src,
        K_src,
        vadas_model,
        R_fe2src,
        (Wf,Hf),
        store_mode=args.store_mode,
        splat_radius=args.splat_radius,
        hole_fill=not args.no_hole_fill
    )

    valid = depth_fe > 0
    coverage = valid.sum()/depth_fe.size
    if np.any(valid):
        print(f"[INFO] Fisheye depth stats: min={depth_fe[valid].min():.3f} max={depth_fe.max():.3f} mean={depth_fe[valid].mean():.3f}")
    print(f"[INFO] Coverage={coverage:.3f}")

    inv = np.zeros_like(depth_fe)
    inv[valid] = 1.0 / np.maximum(depth_fe[valid], 1e-6)

    out_dir = Path(args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    cv.imwrite(str(out_dir/"depth.png"), colorize(depth_fe, (0,0,0)))
    cv.imwrite(str(out_dir/"inverse_depth.png"), colorize(inv, (255,255,255)))
    hole_vis = np.zeros((Hf,Wf,3), np.uint8)
    hole_vis[valid] = (0,255,0); hole_vis[~valid] = (0,0,255)
    cv.imwrite(str(out_dir/"holes.png"), hole_vis)
    np.save(out_dir/"depth.npy", depth_fe)
    np.save(out_dir/"inverse_depth.npy", inv)
    print(f"[SAVE] Outputs -> {out_dir}")

# ---------------- CLI ----------------
def build_argparser():
    ap = argparse.ArgumentParser(description="Test: Pinhole depth -> Fisheye depth reprojection")
    ap.add_argument("--image_path", type=str, required=False,
                    default=r"C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\output\distribution_analysis_0000000931\matched_depth_0000000931.png")
    ap.add_argument("--out_dir", type=str, default="./output/test_reproject")
    ap.add_argument("--out_size", type=str, default="1920x1536", help="WxH fisheye output size")
    ap.add_argument("--src_fov_deg", type=float, default=90.0, help="Used if fx/fy not provided")
    ap.add_argument("--fx", type=float, default=None)
    ap.add_argument("--fy", type=float, default=None)
    ap.add_argument("--cx", type=float, default=None)
    ap.add_argument("--cy", type=float, default=None)
    ap.add_argument("--store_mode", choices=["range","forward"], default="range")
    ap.add_argument("--splat_radius", type=int, default=2)
    ap.add_argument("--no_hole_fill", action="store_true")
    args = ap.parse_args([]) if "__file__" not in globals() else ap.parse_args()
    w,h = map(int, args.out_size.lower().split("x"))
    args.out_size = (w,h)
    return args

if __name__ == "__main__":
    args = build_argparser()
    run(args)