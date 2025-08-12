import argparse, json
from pathlib import Path
import numpy as np
import cv2 as cv
from PIL import Image
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
from ref.ref_calibration_data import DEFAULT_CALIB
from ref.vadas_undistortion_utils import get_vadas_undistortion_maps

@torch.inference_mode()
def run_depth_anything_v2(rgb_pil, model_id="depth-anything/Depth-Anything-V2-Base-hf", device=None, half=False):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] Using device: {device}") # ADDED: Print selected device
    processor = AutoImageProcessor.from_pretrained(model_id, use_fast=False)
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
    ap.add_argument("--out",   required=True, help="Output prefix (e.g., ./out/result)")
    ap.add_argument("--model", default="depth-anything/Depth-Anything-V2-Base-hf",
                    help="HF model id (Small/Base/Large): depth-anything/Depth-Anything-V2-*-hf")
    ap.add_argument("--balance", type=float, default=0.0, help="Undistort balance [0..1]")
    ap.add_argument("--fov-scale", type=float, default=1.0, help="Scale new_K focal (<1 wider)")
    ap.add_argument("--out-width", type=int, default=0, help="Rectified width (0→input W)")
    ap.add_argument("--out-height", type=int, default=0, help="Rectified height (0→input H)")
    ap.add_argument("--rectified-fov-scale", type=float, default=1.0, help="Scale factor for rectified image FOV (<1.0 for wider FOV)")
    ap.add_argument("--device", default=None, help="cuda | cpu (default: auto)")
    ap.add_argument("--fp16", action="store_true", help="Use half precision on CUDA")
    ap.add_argument("--no-back-project", action="store_true", help="Skip mapping depth back to fisheye grid")
    ap.add_argument("--no-undistort", action="store_true", help="Skip undistortion and use raw fisheye image for depth inference")
    args = ap.parse_args()

    # --- Load & Process Image ---
    bgr_fish = cv.imread(args.image, cv.IMREAD_COLOR)
    if bgr_fish is None:
        raise FileNotFoundError(args.image)

    Hs, Ws = bgr_fish.shape[:2]

    if args.no_undistort:
        print("[INFO] Skipping undistortion. Using raw fisheye image for depth inference.")
        bgr_rect = bgr_fish # Use raw fisheye image as rectified
        Hr, Wr = Hs, Ws
        # If no undistortion, no back-projection is meaningful
        args.no_back_project = True
    else:
        # VADAS 캘리브레이션 데이터 로드
        vadas_intrinsic = DEFAULT_CALIB["a6"]["intrinsic"]
        original_image_size = (Ws, Hs)

        rect_size = (args.out_width or Ws, args.out_height or Hs)
        
        # VADAS 모델을 사용하여 왜곡 보정 맵 생성
        map1, map2 = get_vadas_undistortion_maps(
            vadas_intrinsic,
            original_image_size,
            rectified_size=rect_size,
            rectified_fov_scale=args.rectified_fov_scale
        )

        # cv.remap을 사용하여 이미지 왜곡 보정
        bgr_rect = cv.remap(bgr_fish, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
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
        # [ADDED] --- Back-project rectified depth to fisheye grid
        # map1, map2는 이미 get_vadas_undistortion_maps에서 생성됨
        # valid 마스크는 map1, map2의 유효하지 않은 값(-1)을 통해 생성
        valid = ((map1 != -1) & (map2 != -1)).astype(np.float32)

        # [ADDED] --- Back-project rectified depth to fisheye grid
        depth_fish01 = cv.remap(
            depth_rect01.astype(np.float32), map1, map2,
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
