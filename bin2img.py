import os
import numpy as np
from PIL import Image


BASE_ROOT = r"E:\ncdb_indoor_temp"  # 루트 경로 (여기 아래의 record_img만 처리)
WIDTH, HEIGHT, CHANNELS = 1920, 1536, 3  # 기대 해상도


def convert_bin(bin_path: str, dst_dir: str) -> None:
    """CH3 *.bin 하나를 dst_dir에 같은 이름으로 .jpg 저장 (원본 보존)."""
    if os.path.getsize(bin_path) == 0:
        print(f"⚠️ Skipping empty file: {bin_path}")
        return

    try:
        data = np.fromfile(bin_path, dtype=np.uint8)
        expected = WIDTH * HEIGHT * CHANNELS
        if data.size != expected:
            print(f"❌ Invalid size: {bin_path} (expected {expected}, got {data.size})")
            return

        img_array = data.reshape((HEIGHT, WIDTH, CHANNELS))
        img = Image.fromarray(img_array, "RGB")

        os.makedirs(dst_dir, exist_ok=True)
        dst_path = os.path.join(dst_dir, os.path.splitext(os.path.basename(bin_path))[0] + ".jpg")
        img.save(dst_path)
        print(f"✅ Saved: {dst_path}")
    except Exception as e:
        print(f"❗ Error processing {bin_path}: {e}")


def process_record_dir(record_dir: str) -> None:
    """record_img 디렉토리 하나를 스캔하여 CH3 포함 파일만 변환."""
    dst_dir = os.path.join(os.path.dirname(record_dir), "ch3")
    for fname in sorted(os.listdir(record_dir)):
        if not fname.lower().endswith(".bin"):
            continue
        if "ch3" not in fname.lower():
            continue
        bin_path = os.path.join(record_dir, fname)
        convert_bin(bin_path, dst_dir)


def main() -> None:
    print(f"Scanning for record_img under: {BASE_ROOT}")
    for root, dirs, _ in os.walk(BASE_ROOT):
        for d in dirs:
            if d.lower() == "record_img":
                record_dir = os.path.join(root, d)
                print(f"[DIR] {record_dir}")
                process_record_dir(record_dir)


if __name__ == "__main__":
    main()