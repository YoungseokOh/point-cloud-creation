# filepath: c:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\compare_depth_maps.py
import numpy as np
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
from typing import Tuple
import argparse
import sys
import json  # JSON 저장을 위해 추가

# 기본 synced_data 경로 (필요 시 --synced_data로 덮어쓰기)
DEFAULT_SYNCED_DATA = Path(r"D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-39-30_243127_B\synced_data")

def normalize_frame_id(frame_id: str) -> str:
    """'1145' 같은 입력도 10자리 '0000001145'로 zero-pad."""
    stem = frame_id.strip()
    if stem.lower().endswith((".png", ".jpg")):
        stem = Path(stem).stem
    if not stem.isdigit():
        raise ValueError(f"frame id must contain only digits. given: '{frame_id}'")
    return stem.zfill(10)

def resolve_paths_from_id(frame_id: str, synced_data_dir: Path) -> Tuple[Path, Path]:
    """10자리 id로 두 PNG 경로 생성."""
    fid = normalize_frame_id(frame_id)
    d1 = synced_data_dir / "depth_maps" / f"{fid}.png"
    d2 = synced_data_dir / "newest_depth_maps" / f"{fid}.png"
    return d1, d2

def save_figure(fig_path: Path, dpi=150):
    """Matplotlib figure를 저장하고 닫는 헬퍼 함수."""
    plt.tight_layout()
    plt.savefig(fig_path, dpi=dpi, format='jpeg')
    plt.close()

def compare_depth_maps(depth1_path: Path, depth2_path: Path, output_dir: Path = None):
    """
    두 Depth Map을 상세히 비교하고, 모든 분석 결과를 직관적인 JPG 이미지로 저장합니다.
    - depth1: pcd_ori (원본)
    - depth2: pcd++ (결과)
    """
    # 1. 데이터 로드 및 전처리
    raw1 = cv2.imread(str(depth1_path), cv2.IMREAD_UNCHANGED)
    raw2 = cv2.imread(str(depth2_path), cv2.IMREAD_UNCHANGED)
    if raw1 is None or raw2 is None:
        print(f"Error: Depth map load failed. Check paths:\n  - {depth1_path}\n  - {depth2_path}", file=sys.stderr)
        return

    d1 = raw1.astype(np.float32) / 256.0
    d2 = raw2.astype(np.float32) / 256.0

    # 2. 픽셀 유형별 마스크 생성
    common_mask = (d1 > 0) & (d2 > 0)
    lost_mask = (d1 > 0) & (d2 == 0)
    new_mask = (d1 == 0) & (d2 > 0)

    # 3. 각 영역별 데이터 추출 및 통계 분석
    print("--- Comprehensive Analysis ---")
    def _summarize(name, data, unit="m"):
        if data.size == 0:
            print(f"{name}: count=0")
            return data
        print(f"{name}: count={data.size} | "
              f"mean={data.mean():.3f}{unit}, std={data.std():.3f}{unit}, "
              f"min={data.min():.3f}{unit}, max={data.max():.3f}{unit}")
        return data

    d1_all = _summarize("d1 (pcd_ori) All Valid Depths", d1[d1 > 0])
    d2_all = _summarize("d2 (pcd++) All Valid Depths", d2[d2 > 0])
    
    lost_depths = _summarize("Lost Pixels Depths", d1[lost_mask])
    new_depths = _summarize("New Pixels Depths", d2[new_mask])
    
    common_diff = d1[common_mask] - d2[common_mask]
    _summarize("Common Pixels Diff", common_diff)

    # 4. 시각화 및 저장 준비
    if not output_dir:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nSaving all analysis images to: {output_dir.resolve()}")

    # 시각화용 전역 변수
    v_depth = np.percentile(d2_all, 98) if d2_all.size > 0 else 10.0
    def make_cmap(name):
        cm = plt.get_cmap(name).copy()
        cm.set_bad(color='white')
        return cm
    jet_cm, seismic_cm, magma_cm = make_cmap('jet'), make_cmap('seismic'), make_cmap('magma')

    # 5. 모든 분석 이미지 생성 및 저장
    # (5.1) 기본 뎁스맵
    plt.imsave(output_dir / "pcd_ori_colormap.jpg", np.ma.masked_where(d1==0, d1), cmap=jet_cm, vmin=0, vmax=v_depth)
    plt.imsave(output_dir / "pcdpp_colormap.jpg", np.ma.masked_where(d2==0, d2), cmap=jet_cm, vmin=0, vmax=v_depth)

    # (5.2) 픽셀 변화 시각화
    presence_rgb = np.full((*d1.shape, 3), 255, np.uint8)
    presence_rgb[lost_mask] = (0, 0, 255)   # Blue
    presence_rgb[new_mask] = (255, 0, 0)   # Red
    presence_rgb[common_mask]  = (128, 0, 128) # Purple
    cv2.imwrite(str(output_dir / "presence_map.jpg"), cv2.cvtColor(presence_rgb, cv2.COLOR_RGB2BGR))

    total_diff_map = d1 - d2
    v_total_diff = np.percentile(np.abs(total_diff_map[total_diff_map != 0]), 98) if np.any(total_diff_map) else 1.0
    plt.imsave(output_dir / "total_difference_map.jpg", total_diff_map, cmap=seismic_cm, vmin=-v_total_diff, vmax=v_total_diff)

    # (5.3) 각 영역별 상세 뎁스맵
    plt.imsave(output_dir / "new_pixels_depth.jpg", np.ma.masked_where(~new_mask, d2), cmap=jet_cm, vmin=0, vmax=v_depth)
    plt.imsave(output_dir / "lost_pixels_depth.jpg", np.ma.masked_where(~lost_mask, d1), cmap=jet_cm, vmin=0, vmax=v_depth)
    
    common_abs_diff_map = np.zeros_like(d1)
    common_abs_diff_map[common_mask] = np.abs(common_diff)
    v_common_abs = np.percentile(common_abs_diff_map[common_mask], 95) if common_diff.size > 0 else 0.1
    plt.imsave(output_dir / "common_pixels_abs_diff.jpg", np.ma.masked_where(~common_mask, common_abs_diff_map), cmap=magma_cm, vmin=0, vmax=max(v_common_abs, 1e-4))

    # (5.4) 히스토그램
    if d1_all.size > 0:
        plt.figure(); plt.hist(d1_all, bins=100, color='steelblue'); plt.title("d1 (pcd_ori) Depth Distribution"); plt.xlabel("Depth (m)"); save_figure(output_dir / "hist_pcd_ori_depth.jpg")
    if d2_all.size > 0:
        plt.figure(); plt.hist(d2_all, bins=100, color='darkorange'); plt.title("d2 (pcd++) Depth Distribution"); plt.xlabel("Depth (m)"); save_figure(output_dir / "hist_pcdpp_depth.jpg")
    if new_depths.size > 0:
        plt.figure(); plt.hist(new_depths, bins=50, color='tomato'); plt.title("New Pixels Depth Distribution"); plt.xlabel("Depth (m)"); save_figure(output_dir / "hist_new_pixels_depth.jpg")
    if lost_depths.size > 0:
        plt.figure(); plt.hist(lost_depths, bins=50, color='royalblue'); plt.title("Lost Pixels Depth Distribution"); plt.xlabel("Depth (m)"); save_figure(output_dir / "hist_lost_pixels_depth.jpg")
    if common_diff.size > 0:
        plt.figure(); plt.hist(common_diff, bins=50); plt.title("Common Pixels Difference (d1-d2)"); plt.xlabel("Depth Diff (m)"); save_figure(output_dir / "hist_common_pixels_diff.jpg")

    # (5.5) 종합 비교 패널
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes[0, 0].imshow(d1, cmap=jet_cm, vmin=0, vmax=v_depth); axes[0, 0].set_title("d1 (pcd_ori)")
    axes[0, 1].imshow(d2, cmap=jet_cm, vmin=0, vmax=v_depth); axes[0, 1].set_title("d2 (pcd++)")
    axes[1, 0].imshow(presence_rgb); axes[1, 0].set_title("Presence (R:New, B:Lost, P:Common)")
    im = axes[1, 1].imshow(total_diff_map, cmap=seismic_cm, vmin=-v_total_diff, vmax=v_total_diff); axes[1, 1].set_title("Total Diff (d1-d2)")
    fig.colorbar(im, ax=axes[1, 1], orientation='vertical', fraction=0.046, pad=0.04)
    for ax in axes.flat: ax.axis('off')
    save_figure(output_dir / "comparison_overview.jpg", dpi=200)
    
    print("\nAll analysis files saved successfully.")

def analyze_folder(synced_data_dir: Path, output_dir: Path):
    """
    폴더 내 모든 프레임을 분석하여 통계 수집 및 JSON 저장.
    """
    depth_maps_dir = synced_data_dir / "depth_maps"
    newest_depth_maps_dir = synced_data_dir / "newest_depth_maps"
    
    if not depth_maps_dir.exists() or not newest_depth_maps_dir.exists():
        print(f"Error: Depth map directories not found:\n  - {depth_maps_dir}\n  - {newest_depth_maps_dir}", file=sys.stderr)
        return
    
    stats = {
        "total_frames": 0,
        "frames_with_lost_pixels": 0,
        "frames_with_common_diff_zero": 0,
        "frames_with_non_zero_common_diff": [],  # Common Diff가 0이 아닌 프레임 ID 리스트 추가
        "frames": {},
        "summary": {}
    }
    
    all_lost_counts = []
    all_common_diffs = []
    
    for d1_path in depth_maps_dir.glob("*.png"):
        fid = d1_path.stem
        d2_path = newest_depth_maps_dir / f"{fid}.png"
        
        if not d2_path.exists():
            continue
        
        stats["total_frames"] += 1
        
        # 분석 수행 (이미지 저장 없이)
        raw1 = cv2.imread(str(d1_path), cv2.IMREAD_UNCHANGED)
        raw2 = cv2.imread(str(d2_path), cv2.IMREAD_UNCHANGED)
        if raw1 is None or raw2 is None:
            continue
        
        d1 = raw1.astype(np.float32) / 256.0
        d2 = raw2.astype(np.float32) / 256.0
        
        common_mask = (d1 > 0) & (d2 > 0)
        lost_mask = (d1 > 0) & (d2 == 0)
        new_mask = (d1 == 0) & (d2 > 0)
        
        lost_count = np.sum(lost_mask)
        new_count = np.sum(new_mask)
        common_count = np.sum(common_mask)
        
        common_diff = d1[common_mask] - d2[common_mask]
        common_diff_mean = float(common_diff.mean()) if common_diff.size > 0 else 0.0
        common_diff_std = float(common_diff.std()) if common_diff.size > 0 else 0.0
        
        # 프레임별 통계 저장
        stats["frames"][fid] = {
            "d1_pixels": int(np.sum(d1 > 0)),
            "d2_pixels": int(np.sum(d2 > 0)),
            "lost_pixels": int(lost_count),
            "new_pixels": int(new_count),
            "common_pixels": int(common_count),
            "common_diff_mean": common_diff_mean,
            "common_diff_std": common_diff_std
        }
        
        all_lost_counts.append(lost_count)
        all_common_diffs.extend(common_diff.tolist())
        
        if lost_count > 0:
            stats["frames_with_lost_pixels"] += 1
        
        # Common Diff가 0에 가까운지 확인 및 리스트 추가
        if abs(common_diff_mean) < 1e-6 and common_diff_std < 1e-6:
            stats["frames_with_common_diff_zero"] += 1
        else:
            stats["frames_with_non_zero_common_diff"].append(fid)  # 조건 만족하지 않으면 리스트에 추가
    
    # 전체 요약 계산
    stats["summary"] = {
        "total_frames": stats["total_frames"],
        "frames_with_lost_pixels": stats["frames_with_lost_pixels"],
        "frames_with_common_diff_zero": stats["frames_with_common_diff_zero"],
        "frames_with_non_zero_common_diff": stats["frames_with_non_zero_common_diff"],  # 요약에 포함
        "avg_lost_pixels": float(np.mean(all_lost_counts)) if all_lost_counts else 0.0,
        "avg_common_diff_mean": float(np.mean(all_common_diffs)) if all_common_diffs else 0.0,
        "avg_common_diff_std": float(np.std(all_common_diffs)) if all_common_diffs else 0.0,
        "verification": {
            "all_lost_pixels_zero": stats["frames_with_lost_pixels"] == 0,
            "all_common_diff_zero": stats["frames_with_common_diff_zero"] == stats["total_frames"]
        }
    }
    
    # JSON 저장
    output_file = output_dir / "folder_stats.json"
    with open(output_file, 'w') as f:
        json.dump(stats, f, indent=4)
    
    print(f"Folder analysis complete. Stats saved to {output_file}")
    print(f"Summary: {stats['summary']}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare two depth maps and save detailed analysis as JPG images, or analyze entire folder for stats.")
    parser.add_argument("--id", type=str, required=False, help="Frame id (e.g., 1145). Will be zero-padded. Analyzes one frame and saves JPGs.")
    parser.add_argument("--folder", type=str, required=False, help="Path to synced_data folder for full analysis. Saves stats as JSON only.")  # --folder 옵션 추가
    parser.add_argument("--synced_data", type=str, default=str(DEFAULT_SYNCED_DATA), help="Path to synced_data folder (used with --id).")
    parser.add_argument("--depth1", type=str, required=False, help="Full path to pcd_ori PNG. Overrides --id.")
    parser.add_argument("--depth2", type=str, required=False, help="Full path to pcd++ PNG. Overrides --id.")
    parser.add_argument("--output", type=str, default=r"output\compare_depth_results", help="Output directory for JPG analysis files.")
    
    args = parser.parse_args()
    
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    if args.folder:
        synced_data_dir = Path(args.folder)
        analyze_folder(synced_data_dir, output_dir)
    elif args.id or (args.depth1 and args.depth2):
        if args.depth1 and args.depth2:
            depth1_path, depth2_path = Path(args.depth1), Path(args.depth2)
        elif args.id:
            depth1_path, depth2_path = resolve_paths_from_id(args.id, Path(args.synced_data))
        else:
            parser.error("Provide --id or both --depth1 and --depth2.")
        
        if not depth1_path.exists() or not depth2_path.exists():
            print(f"Error: Input file(s) not found.\n  - {depth1_path}\n  - {depth2_path}", file=sys.stderr)
            sys.exit(1)
        
        compare_depth_maps(depth1_path, depth2_path, output_dir)
    else:
        parser.error("Provide --id/--depth1+--depth2 for single analysis, or --folder for full analysis.")
