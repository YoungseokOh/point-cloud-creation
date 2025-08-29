# depth_infer_raw.py
# [SIMPLIFIED] 원본 이미지에 바로 Depth Anything V2 추론 → raw .npy 및 시각화 저장

import argparse
from pathlib import Path
import numpy as np
import cv2 as cv
from PIL import Image
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

@torch.inference_mode()
def run_depth_anything_v2(rgb_pil,
                          model_id="depth-anything/Depth-Anything-V2-Base-hf",
                          device=None,
                          half=False):
    """
    원본 RGB(PIL) 이미지를 입력으로 Depth Anything V2 추론.
    반환:
      depth01: (H,W) [0,1] 정규화 깊이 (시각화용)
      raw_depth_tensor: (1,H,W) 원시 깊이 텐서 (모델 출력)
    """
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] Using device: {device}")

    processor = AutoImageProcessor.from_pretrained(model_id, use_fast=False)
    model = AutoModelForDepthEstimation.from_pretrained(model_id).to(device)
    if half and device == "cuda":
        model = model.half()

    inputs = processor(images=rgb_pil, return_tensors="pt")
    if half and device == "cuda":
        inputs = {k: (v.half() if isinstance(v, torch.Tensor) else v) for k, v in inputs.items()}
    inputs = {k: (v.to(device) if isinstance(v, torch.Tensor) else v) for k, v in inputs.items()}

    outputs = model(**inputs)

    # HF processor로 원본 해상도로 후처리
    post = processor.post_process_depth_estimation(
        outputs, target_sizes=[(rgb_pil.height, rgb_pil.width)]
    )[0]["predicted_depth"]  # (H,W) torch.Tensor

    # 시각화를 위해 [0,1] 정규화
    d = post.detach().float()
    d01 = (d - d.min()) / (d.max() - d.min() + 1e-8)

    # raw 텐서는 (1,H,W) 모양이 일반적이므로 맞춰서 반환
    # raw_depth_tensor를 post_process_depth_estimation을 거친 'post' 변수로 대체
    # 'post'는 이미 원본 이미지 크기로 조정되어 있습니다.
    # save_depth_products에서 (1, H, W) 형태를 기대하므로 unsqueeze(0)를 해줍니다.
    raw_depth_tensor_resized = post.unsqueeze(0)
    print(f"[DEBUG] Raw Depth Tensor Shape (after resize to original): {raw_depth_tensor_resized.shape}")
    return d01.cpu().numpy(), raw_depth_tensor_resized

def save_depth_products(depth01, out_prefix, raw_depth_tensor=None):
    """
    저장 산출물:
      - {out_prefix}.png            : 8-bit 정규화 깊이
      - {out_prefix}.16bit.png      : 16-bit 정규화 깊이
      - {out_prefix}.jet.png        : JET 컬러맵
      - {out_prefix}.raw.npy        : 원시 모델 출력 (정규화 전, float32 저장)
      - {out_prefix}.raw.png        : 원시 모델 출력의 16-bit 시각화
    반환:
      d8(8-bit), jet(BGR)
    """
    out_prefix = Path(out_prefix)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    # [0,1] → 8/16-bit
    d8 = (depth01 * 255.0).clip(0, 255).astype(np.uint8)
    d16 = (depth01 * 65535.0).clip(0, 65535).astype(np.uint16)

    cv.imwrite(str(out_prefix.with_suffix(".png")), d8)
    cv.imwrite(str(out_prefix.with_suffix(".16bit.png")), d16)

    jet = cv.applyColorMap(d8, cv.COLORMAP_JET)
    cv.imwrite(str(out_prefix.with_suffix(".jet.png")), jet)

    # raw 저장 (정규화 없이 .npy) + 16-bit 시각화 png
    if raw_depth_tensor is not None:
        raw_np = raw_depth_tensor.detach().cpu().numpy()
        # 예상 모양: (1, H, W) → (H, W)
        if raw_np.ndim == 3 and raw_np.shape[0] == 1:
            raw_np = raw_np.squeeze(0)

        # npy 저장 (float32 권장)
        np.save(str(out_prefix.with_suffix(".raw.npy")), raw_np.astype(np.float32))

        # 16-bit png 시각화용 min-max 정규화
        rmin, rmax = raw_np.min(), raw_np.max()
        if rmax - rmin > 1e-8:
            raw_png = ((raw_np - rmin) / (rmax - rmin) * 65535.0).clip(0, 65535).astype(np.uint16)
        else:
            raw_png = np.zeros_like(raw_np, dtype=np.uint16)
        cv.imwrite(str(out_prefix.with_suffix(".raw.png")), raw_png)

    return d8, jet

def save_overlay(bgr, depth01, out_path, alpha=0.55):
    """
    입력 BGR 위에 깊이 컬러맵 오버레이 저장
    """
    d8 = (depth01 * 255.0).clip(0, 255).astype(np.uint8)
    jet = cv.applyColorMap(d8, cv.COLORMAP_JET)
    overlay = cv.addWeighted(bgr, 1 - alpha, jet, alpha, 0)
    cv.imwrite(str(out_path), overlay)

def main():
    ap = argparse.ArgumentParser(description="Run Depth Anything V2 on the raw image and save raw/visualizations")
    ap.add_argument("--image", required=True, help="Input image path")
    ap.add_argument("--out", required=True, help="Output prefix, e.g., ./out/result")
    ap.add_argument("--model", default="depth-anything/Depth-Anything-V2-Base-hf",
                    help="HF model id (Small/Base/Large), e.g., depth-anything/Depth-Anything-V2-Base-hf")
    ap.add_argument("--device", default=None, help="cuda | cpu (default: auto)")
    ap.add_argument("--fp16", action="store_true", help="Use half precision on CUDA")
    args = ap.parse_args()

    # 원본 이미지 로드
    bgr = cv.imread(args.image, cv.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(args.image)
    print(f"[DEBUG] Input image shape: {bgr.shape}")

    # RGB(PIL)로 변환 후 추론
    rgb = cv.cvtColor(bgr, cv.COLOR_BGR2RGB)
    depth01, raw_depth_tensor = run_depth_anything_v2(
        Image.fromarray(rgb),
        model_id=args.model,
        device=args.device,
        half=args.fp16
    )

    # 저장: 정규화 깊이, JET, raw .npy/.png
    out_prefix = Path(args.out).with_suffix("")  # 확장자 제거한 prefix
    d8, jet = save_depth_products(depth01, out_prefix, raw_depth_tensor=raw_depth_tensor)

    # 오버레이 저장
    save_overlay(bgr, depth01, out_prefix.with_suffix(".overlay.png"), alpha=0.55)

    print("[DONE] Saved outputs under:", out_prefix.parent)

if __name__ == "__main__":
    main()