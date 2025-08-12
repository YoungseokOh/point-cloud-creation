import argparse, json
from pathlib import Path
import numpy as np
import cv2 as cv
from PIL import Image
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

def load_calib(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)
    K = np.array(data["K"], dtype=np.float64)
    D = np.array(data["D"], dtype=np.float64).reshape(-1, 1)
    if D.size not in (4, 8, 12):
        raise ValueError("fisheye D should have 4 coeffs (k1..k4).")
    img_size = tuple(data.get("image_size", (0, 0)))
    return K, D, img_size

def fisheye_undistort_bilinear(bgr, K, D, out_size=None, balance=0.0, fov_scale=1.0):
    """
    Fisheye -> Rectified(Bilinear). 반환: (undistorted_bgr, new_K)
    - balance: [0..1] (0=crop, 1=FOV 유지)
    - fov_scale: 새 초점 스케일 ( <1 넓게, >1 좁게 )
    """
    h, w = bgr.shape[:2]
    if out_size is None:
        out_size = (w, h)
    new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), np.eye(3), balance=balance, new_size=out_size
    )
    new_K[0, 0] *= fov_scale
    new_K[1, 1] *= fov_scale

    map1, map2 = cv.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_K, out_size, cv.CV_16SC2
    )
    undistorted = cv.remap(bgr, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    return undistorted, new_K

@torch.inference_mode()
def run_depth_anything_v2(rgb_pil, model_id="depth-anything/Depth-Anything-V2-Base-hf", device=None, half=False):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    processor = AutoImageProcessor.from_pretrained(model_id)
    model = AutoModelForDepthEstimation.from_pretrained(model_id).to(device)
    if half and device == "cuda":
        model = model.half()

    inputs = processor(images=rgb_pil, return_tensors="pt")
    if half and device == "cuda":
        inputs = {k: (v.half() if isinstance(v, torch.Tensor) else v) for k, v in inputs.items()}
    inputs = {k: (v.to(device) if isinstance(v, torch.Tensor) else v) for k, v in inputs.items()}

    outputs = model(**inputs)
    post = processor.post_process_depth_estimation(
        outputs, target_sizes=[(rgb_pil.height, rgb_pil.width)]
    )[0]["predicted_depth"]
    d = post.detach().float()
    d = (d - d.min()) / (d.max() - d.min() + 1e-8)  # per-image [0,1]
    return d.cpu().numpy()

def save_depth_maps(depth01, out_prefix):
    """
    저장:
      - out_prefix.png           : 8-bit depth (미리보기)
      - out_prefix.16bit.png     : 16-bit depth (상대깊이)
      - out_prefix.jet.png       : JET 컬러맵
    """
    out_prefix = Path(out_prefix)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    d8 = (depth01 * 255.0).clip(0, 255).astype(np.uint8)
    d16 = (depth01 * 65535.0).clip(0, 65535).astype(np.uint16)
    cv.imwrite(str(out_prefix.with_suffix(".png")), d8)
    cv.imwrite(str(out_prefix.with_suffix(".16bit.png")), d16)
    jet = cv.applyColorMap(d8, cv.COLORMAP_JET)
    cv.imwrite(str(out_prefix.with_suffix(".jet.png")), jet)
    return d8, jet  # [ADDED] 후속 오버레이용

# [ADDED] fisheye 그리드(원본) -> rectified 평면 좌표 맵 생성
def build_fisheye_to_rect_map(K, D, new_K, src_size, rect_size):
    """
    src_size: (W_src, H_src)  (원본 fisheye)
    rect_size: (W_rect, H_rect) (rectified)
    반환: mapx, mapy, valid(0/1 float32)
    """
    W_s, H_s = src_size
    W_r, H_r = rect_size

    # (u,v) grid on fisheye image
    uu, vv = np.meshgrid(np.arange(W_s, dtype=np.float32),
                         np.arange(H_s, dtype=np.float32))
    pts = np.stack([uu, vv], axis=-1).reshape(-1, 1, 2)  # (N,1,2)

    # 각 fisheye 픽셀이 rectified 평면에서 어디로 가는지 (픽셀 좌표) 계산
    rect_pts = cv.fisheye.undistortPoints(
        pts, K, D, R=np.eye(3), P=new_K
    ).reshape(H_s, W_s, 2).astype(np.float32)

    mapx = rect_pts[..., 0]
    mapy = rect_pts[..., 1]

    # 유효한 좌표 (rectified 이미지 범위 안)
    valid = ((mapx >= 0) & (mapx < (W_r - 1)) &
             (mapy >= 0) & (mapy < (H_r - 1))).astype(np.float32)

    return mapx, mapy, valid

# [ADDED] Overlay & Mosaic helpers
def make_overlay(bgr, depth01, valid_mask=None, alpha=0.55):
    d8 = (depth01 * 255.0).clip(0, 255).astype(np.uint8)
    jet = cv.applyColorMap(d8, cv.COLORMAP_JET)
    if valid_mask is not None:
        vm = (valid_mask * 255).astype(np.uint8)
        jet = cv.bitwise_and(jet, jet, mask=vm)
    overlay = cv.addWeighted(bgr, 1 - alpha, jet, alpha, 0)
    return overlay, jet, d8

def save_mosaic(path, tiles, scale_to_height=None):
    """
    tiles: [(img_bgr, title_str), ...]  가로로 이어붙인 뒤 세로 스택
    scale_to_height: 모든 타일을 동일 높이로 리사이즈(옵션)
    """
    rows = []
    for row in tiles:
        imgs = []
        h_target = scale_to_height or min(img.shape[0] for img, _ in row)
        for img, title in row:
            # 높이 기준 리사이즈
            h, w = img.shape[:2]
            if h != h_target:
                w_new = int(w * (h_target / h))
                img = cv.resize(img, (w_new, h_target), interpolation=cv.INTER_AREA)
            # 타이틀 라벨 얇게 추가
            pad = 36
            canvas = np.zeros((img.shape[0] + pad, img.shape[1], 3), np.uint8)
            canvas[:img.shape[0], :, :] = img
            cv.putText(canvas, title, (10, img.shape[0] + 26),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv.LINE_AA)
            imgs.append(canvas)
        rows.append(cv.hconcat(imgs))
    mosaic = cv.vconcat(rows)
    cv.imwrite(str(path), mosaic)

def main():
    ap = argparse.ArgumentParser(description="Fisheye → Rectified → Depth Anything V2 → Back-project to Fisheye")
    ap.add_argument("--image", required=True, help="Input fisheye image path")
    ap.add_argument("--calib", required=True, help="Calibration JSON (K,D,image_size)")
    ap.add_argument("--out",   required=True, help="Output prefix (e.g., ./out/result)")
    ap.add_argument("--model", default="depth-anything/Depth-Anything-V2-Base-hf",
                    help="HF model id (Small/Base/Large): depth-anything/Depth-Anything-V2-*-hf")
    ap.add_argument("--balance", type=float, default=0.0, help="Undistort balance [0..1]")
    ap.add_argument("--fov-scale", type=float, default=1.0, help="Scale new_K focal (<1 wider)")
    ap.add_argument("--out-width", type=int, default=0, help="Rectified width (0→input W)")
    ap.add_argument("--out-height", type=int, default=0, help="Rectified height (0→input H)")
    ap.add_argument("--device", default=None, help="cuda | cpu (default: auto)")
    ap.add_argument("--fp16", action="store_true", help="Use half precision on CUDA")
    ap.add_argument("--no-back-project", action="store_true", help="Skip mapping depth back to fisheye grid")
    args = ap.parse_args()

    # --- Load & Rectify
    bgr_fish = cv.imread(args.image, cv.IMREAD_COLOR)
    if bgr_fish is None:
        raise FileNotFoundError(args.image)

    K, D, img_size = load_calib(args.calib)
    Hs, Ws = bgr_fish.shape[:2]
    if img_size != (0, 0) and tuple(img_size) != (Ws, Hs):
        print(f"[WARN] calib image_size {img_size} != input image size {(Ws,Hs)}; proceeding anyway.")

    rect_size = (args.out_width or Ws, args.out_height or Hs)
    bgr_rect, new_K = fisheye_undistort_bilinear(
        bgr_fish, K, D, out_size=rect_size, balance=args.balance, fov_scale=args.fov_scale
    )
    Hr, Wr = bgr_rect.shape[:2]

    # --- Depth Inference on Rectified
    rgb_rect = cv.cvtColor(bgr_rect, cv.COLOR_BGR2RGB)
    depth_rect01 = run_depth_anything_v2(
        Image.fromarray(rgb_rect), model_id=args.model, device=args.device, half=args.fp16
    )
    assert depth_rect01.shape == (Hr, Wr), f"Depth shape {depth_rect01.shape} != rectified {(Hr,Wr)}"

    # Save rectified depth (8/16bit + JET)
    rect_prefix = Path(args.out).with_suffix("")  # base prefix
    d8_rect, jet_rect = save_depth_maps(depth_rect01, rect_prefix.with_name(rect_prefix.name + ".rect"))

    # Rectified overlay 저장
    overlay_rect, jet_for_rect, _ = make_overlay(bgr_rect, depth_rect01, valid_mask=None, alpha=0.55)
    cv.imwrite(str(rect_prefix.with_name(rect_prefix.name + ".rect.overlay.png")), overlay_rect)

    if not args.no_back_project:
        # [ADDED] --- Build inverse mapping: fisheye grid → rectified coords
        mapx, mapy, valid = build_fisheye_to_rect_map(
            K, D, new_K, src_size=(Ws, Hs), rect_size=(Wr, Hr)
        )

        # [ADDED] --- Back-project rectified depth to fisheye grid
        depth_fish01 = cv.remap(
            depth_rect01.astype(np.float32), mapx, mapy,
            interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT, borderValue=0
        )
        # 유효 픽셀만 남기기 (나머지 0)
        depth_fish01 *= valid

        # [ADDED] 저장: fisheye 좌표계 depth + mask + overlay
        fish_prefix = rect_prefix.with_name(rect_prefix.name + ".fish")
        d8_fish, jet_fish = save_depth_maps(depth_fish01, fish_prefix)

        valid_u8 = (valid * 255).astype(np.uint8)
        cv.imwrite(str(fish_prefix.with_suffix(".mask.png")), valid_u8)

        overlay_fish, _, _ = make_overlay(bgr_fish, depth_fish01, valid_mask=valid, alpha=0.55)
        cv.imwrite(str(fish_prefix.with_suffix(".overlay.png")), overlay_fish)

        # [ADDED] 모자이크: 시각 점검(원본RGB/RectRGB, FisheyeOverlay/RectOverlay)
        mosaic_path = rect_prefix.with_name(rect_prefix.name + ".mosaic.png")
        save_mosaic(
            mosaic_path,
            tiles=[
                [(bgr_fish, "Fisheye RGB (input)"), (bgr_rect, "Rectified RGB")],
                [(overlay_fish, "Fisheye Overlay (Depth back-projected)"),
                 (overlay_rect, "Rectified Overlay (Depth predicted)")]
            ],
            scale_to_height=min(Hs, Hr)
        )

        # [ADDED] (선택) Rectified RGB를 같은 맵으로 fisheye로 되돌려 비교해보고 싶다면:
        # rect2fish = cv.remap(bgr_rect, mapx, mapy, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
        # cv.imwrite(str(rect_prefix.with_name(rect_prefix.name + ".rect2fish.rgb.png")), rect2fish)

    print("[DONE] Saved outputs under:", rect_prefix.parent)

if __name__ == "__main__":
    main()
