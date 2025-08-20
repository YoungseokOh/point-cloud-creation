import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import csv

# [ADDED] ----- robust affine fit: y ~= s*x + b  (Huber IRLS) -------------------
def robust_affine_fit(x, y, max_iters=30, delta=1.0, eps=1e-6):
    """
    x: source depth (DA), y: target depth (GT). 1D np.array
    delta: Huber 파라미터(표준편차 1.0 기준). 자동 스케일링(mad)과 곱해 사용.
    """
    x = x.astype(np.float64).ravel()
    y = y.astype(np.float64).ravel()
    # 초기값: L2 (lstsq)
    A = np.vstack([x, np.ones_like(x)]).T
    s, b = np.linalg.lstsq(A, y, rcond=None)[0]

    for _ in range(max_iters):
        r = y - (s * x + b)
        # robust scale (MAD)
        mad = np.median(np.abs(r - np.median(r))) + eps
        sigma = 1.4826 * mad + eps
        # Huber weights
        t = np.abs(r) / (delta * sigma)
        w = np.ones_like(t)
        mask = t > 1.0
        w[mask] = 1.0 / t[mask]
        # 가중 최소제곱
        W = np.sqrt(w)
        Aw = A * W[:, None]
        yw = y * W
        s_new, b_new = np.linalg.lstsq(Aw, yw, rcond=None)[0]
        # 수렴 체크
        if np.allclose([s, b], [s_new, b_new], rtol=1e-6, atol=1e-8):
            s, b = s_new, b_new
            break
        s, b = s_new, b_new
    return float(s), float(b)
# -------------------------------------------------------------------------------

def calculate_and_plot_error_by_distance(gt_data, predicted_data, title_prefix, out_dir, file_identifier, suffix):
    """
    거리 구간별 MAE를 계산하고 데이터를 반환합니다.
    """
    valid_pixels_mask = (gt_data > 0) & (predicted_data > 0.1)
    gt_valid = gt_data[valid_pixels_mask]
    predicted_valid = predicted_data[valid_pixels_mask]

    if gt_valid.size == 0 or predicted_valid.size == 0:
        print(f"[INFO] {title_prefix}: 거리별 오차 분석을 위한 유효 데이터가 부족합니다.")
        return [], []

    max_dist = np.max(gt_valid)
    # 0m에서 3m까지는 0.2m 간격, 그 이후는 2m 간격으로 bins 생성
    bins_fine = np.arange(0, min(max_dist, 3.0) + 0.2, 0.2)
    bins_coarse = np.arange(3.0, max_dist + 2, 2)
    bins = np.unique(np.concatenate([bins_fine, bins_coarse]))
    bins = bins[bins <= max_dist + 1] # max_dist를 초과하는 불필요한 bin 제거
    if len(bins) < 2: # 데이터 범위가 너무 좁을 경우 최소 2개의 bin 확보
        bins = np.array([0, max_dist + 1])

    bin_labels = []
    mae_values = []

    for i in range(len(bins) - 1):
        lower_bound, upper_bound = bins[i], bins[i+1]
        bin_mask = (gt_valid >= lower_bound) & (gt_valid < upper_bound)

        if np.sum(bin_mask) > 10: # 최소 10개 이상의 샘플이 있는 경우에만 계산
            gt_bin = gt_valid[bin_mask]
            predicted_bin = predicted_valid[bin_mask]

            abs_error = np.abs(predicted_bin - gt_bin)
            mae = np.mean(abs_error)

            bin_labels.append(f'{lower_bound:.1f}-{upper_bound:.1f}m')
            mae_values.append(mae)

    if bin_labels:
        # Save error metrics to CSV
        csv_output_path = out_dir / f"error_metrics_{suffix}.csv"
        with open(csv_output_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Distance Bin', 'MAE (m)'])
            for i in range(len(bin_labels)):
                csv_writer.writerow([bin_labels[i], mae_values[i]])
        print(f"[SAVE] {title_prefix} 거리별 MAE 지표 저장: {csv_output_path}")
        return bin_labels, mae_values
    else:
        print(f"[INFO] {title_prefix}: 거리별 오차 분석을 위한 데이터가 부족합니다.")
        return [], []

def analyze_and_visualize_distributions():
    """
    Ground Truth, Original DA, Histogram Matched DA 데이터의 분포 및 시각화 차이를 분석합니다.
    """
    # --- 사용자 설정 ---
    depth_anything_model_output_path = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/output/single_image_test.raw.npy")
    existing_depth_map_folder = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/ncdb-cls-sample/synced_data/depth_maps")
    binary_mask_path = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/ncdb-cls-sample/synced_data/binary_mask.png")

    file_identifier = input("비교할 기존 깊이 맵의 10자리 파일명을 입력하세요 (예: 0000000931): ")
    if not file_identifier.isdigit() or len(file_identifier) != 10:
        print("오류: 10자리 숫자로 된 파일명을 입력해야 합니다.")
        return

    matching_threshold_m = 30.0 # 사용자가 25m로 변경 요청

    existing_depth_map_path = existing_depth_map_folder / f"{file_identifier}.png"
    out_dir = Path(f"output/distribution_analysis_{file_identifier}")
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- 데이터 로드 및 전처리 ---
    try:
        da_raw = np.load(str(depth_anything_model_output_path))
        da_raw = np.max(da_raw) - da_raw # Invert
        gt_raw = cv2.imread(str(existing_depth_map_path), cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(str(binary_mask_path), cv2.IMREAD_GRAYSCALE)
        
        if gt_raw is None or mask is None:
            raise FileNotFoundError("GT depth map or mask not found.")

    except Exception as e:
        print(f"데이터 로딩 중 오류 발생: {e}")
        return

    if da_raw.shape != gt_raw.shape:
        gt_raw = cv2.resize(gt_raw, (da_raw.shape[1], da_raw.shape[0]), interpolation=cv2.INTER_NEAREST)
    
    mask_normalized = (mask > 0).astype(np.uint8)
    
    da_masked = np.where(mask_normalized == 1, da_raw.astype(np.float32), 0)
    gt_meters = np.where(mask_normalized == 1, gt_raw.astype(np.float32) / 256.0, 0)

    # --- 히스토그램 매칭 수행 ---
    scaling_mask = (gt_meters > 0.1) & (gt_meters < matching_threshold_m) & (da_masked > 0)
    if np.sum(scaling_mask) < 10:
        print("히스토그램 매칭을 위한 데이터가 부족합니다.")
        return
    
    # [ADDED] s,b 추정
    s_aff, b_aff = robust_affine_fit(da_masked[scaling_mask], gt_meters[scaling_mask])
    affine_da = np.clip(s_aff * da_masked + b_aff, 0, None)

    da_for_matching = da_masked[scaling_mask]
    gt_for_matching = gt_meters[scaling_mask]
    
    valid_da_output = da_masked[da_masked > 0]
    
    sorted_source = np.sort(da_for_matching)
    sorted_template = np.sort(gt_for_matching)
    
    matched_values = np.interp(valid_da_output, sorted_source, sorted_template)
    
    matched_da = np.zeros_like(da_masked)
    matched_da[da_masked > 0] = matched_values

    # --- 유효 데이터 추출 ---
    valid_gt = gt_meters[gt_meters > 0]
    valid_matched = matched_da[matched_da > 0]
    valid_original_da = da_masked[da_masked > 0]

    if valid_gt.size == 0 or valid_matched.size == 0:
        print("분석할 유효 데이터가 없습니다.")
        return

    # --- 시각화 ---
    fig, axes = plt.subplots(2, 2, figsize=(20, 20))
    fig.suptitle(f'Distribution and Visualization Analysis (File: {file_identifier})', fontsize=22)

    # 1. GT 시각화
    vmin_gt, vmax_gt = 0, np.percentile(valid_gt, 49.5)
    im1 = axes[0, 0].imshow(gt_meters, cmap='magma', vmin=vmin_gt, vmax=vmax_gt)
    axes[0, 0].set_title(f'Ground Truth (GT)\nRange: [{vmin_gt:.2f}, {vmax_gt:.2f}] m')
    fig.colorbar(im1, ax=axes[0, 0], fraction=0.046, pad=0.04, label='Depth (m)')

    # 2. Matched DA 시각화
    vmin_matched, vmax_matched = np.percentile(valid_matched, [0.5, 99.5])
    im2 = axes[0, 1].imshow(matched_da, cmap='magma', vmin=vmin_matched, vmax=vmax_matched)
    axes[0, 1].set_title(f'Histogram Matched DA\nRange: [{vmin_matched:.2f}, {vmax_matched:.2f}] m')
    fig.colorbar(im2, ax=axes[0, 1], fraction=0.046, pad=0.04, label='Depth (m)')

    # 3. 히스토그램 비교
    ax_hist = axes[1, 0]
    color_gt = 'lightcoral'
    color_matched = 'mediumpurple'
    ax_hist.set_ylabel('Frequency (Log Scale) - GT / Matched', color=color_gt, fontsize=12)
    
    bins_gt = np.linspace(min(vmin_gt, vmin_matched), max(vmax_gt, vmax_matched), 200)
    ax_hist.hist(valid_gt, bins=bins_gt, color=color_gt, alpha=0.7, label='Ground Truth (Left Axis)')
    ax_hist.hist(valid_matched, bins=bins_gt, color=color_matched, alpha=0.8, label='Matched DA (Left Axis)', histtype='step', linewidth=2.5)
    ax_hist.set_yscale('log')
    ax_hist.tick_params(axis='y', labelcolor=color_gt)
    ax_hist.grid(True, which='both', linestyle='--', linewidth=0.5, axis='y')

    ax_hist_twin = ax_hist.twinx()
    color_orig = 'deepskyblue'
    ax_hist_twin.set_ylabel('Frequency (Log Scale) - Original DA', color=color_orig, fontsize=12)
    
    vmin_orig, vmax_orig = np.percentile(valid_original_da, [0.5, 99.5])
    bins_orig = np.linspace(vmin_orig, vmax_orig, 200)
    ax_hist_twin.hist(valid_original_da, bins=bins_orig, color=color_orig, alpha=0.7, label='Original DA (Right Axis)', histtype='step', linewidth=2.5, linestyle='--')
    ax_hist_twin.set_yscale('log')
    ax_hist_twin.tick_params(axis='y', labelcolor=color_orig)

    ax_hist.set_title('Histogram Comparison (Log Scale)', fontsize=16)
    lines, labels = ax_hist.get_legend_handles_labels()
    lines2, labels2 = ax_hist_twin.get_legend_handles_labels()
    ax_hist_twin.legend(lines + lines2, labels + labels2, loc='upper center')
    
    ax_hist.set_xlabel('Value', fontsize=12)
    ax_hist.text(1.0, -0.12, 'Raw Model Output for Original DA', ha='right', va='top', transform=ax_hist.transAxes, color=color_orig, fontsize=10)
    ax_hist.text(0.0, -0.12, 'Depth (m) for GT & Matched', ha='left', va='top', transform=ax_hist.transAxes, color='black', fontsize=10)

    # [REPLACE] 기존 Q–Q 섹션 전체를 다음으로 교체
    ax_qq = axes[1, 1]
    quantiles = np.linspace(0.01, 0.99, 100)

    # 공통: 유효 마스크(같은 위치 비교를 원하면 overlap 마스크 사용 권장)
    mask_matched = (gt_meters > 0.1) & (matched_da > 0)
    mask_affine  = (gt_meters > 0.1) & (affine_da  > 0)

    # 1) GT vs Histogram Matched
    gt_q_m   = np.quantile(gt_meters[mask_matched], quantiles)
    matched_q = np.quantile(matched_da[mask_matched], quantiles)
    ax_qq.plot(gt_q_m, matched_q, 'o', color='darkgreen', markersize=4, alpha=0.6,
            label='GT vs Matched Quantiles')

    # 2) GT vs Affine (s,b)
    gt_q_a   = np.quantile(gt_meters[mask_affine], quantiles)
    affine_q = np.quantile(affine_da[mask_affine], quantiles)
    ax_qq.plot(gt_q_a, affine_q, '^', color='darkred', markersize=4, alpha=0.6,
            label='GT vs Affine Quantiles')

    # 대각선(y=x) 범위 계산
    lo = min(gt_q_m.min(), gt_q_a.min(), matched_q.min(), affine_q.min())
    hi = max(gt_q_m.max(), gt_q_a.max(), matched_q.max(), affine_q.max())
    ax_qq.plot([lo, hi], [lo, hi], 'k--', linewidth=1.5, label='y = x (Perfect Match)')

    ax_qq.set_xlabel('GT Quantiles (m)', fontsize=12)
    ax_qq.set_ylabel('Predicted Quantiles (m)', fontsize=12)   # [FIX] ylabel 한 번만
    ax_qq.set_title('Q–Q: GT vs Matched / Affine', fontsize=16)
    ax_qq.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax_qq.legend()
    ax_qq.set_aspect('equal', 'box')

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    output_path = out_dir / "fig_distribution_analysis_with_qq.png"
    fig.savefig(output_path, dpi=150)
    print(f"[SAVE] 분석 결과 저장: {output_path}")

    # [ADDED] 스케일 보정된 Depth 맵 저장 (16비트 PNG)
    matched_depth_output_path = out_dir / f"matched_depth_{file_identifier}.png"
    affine_depth_output_path = out_dir / f"affine_depth_{file_identifier}.png"

    cv2.imwrite(str(matched_depth_output_path), (matched_da * 256).astype(np.uint16))
    print(f"[SAVE] Histogram Matched Depth 맵 저장: {matched_depth_output_path}")

    cv2.imwrite(str(affine_depth_output_path), (affine_da * 256).astype(np.uint16))
    print(f"[SAVE] Affine Scaled Depth 맵 저장: {affine_depth_output_path}")

    # --- 거리별 오차 분석 (Figure 6 참고) ---
    print("\n--- 거리별 오차 분석 ---")
    matched_labels, matched_mae = calculate_and_plot_error_by_distance(
        gt_meters, matched_da, "Histogram Matched DA", out_dir, file_identifier, "matched"
    )
    affine_labels, affine_mae = calculate_and_plot_error_by_distance(
        gt_meters, affine_da, "Affine Scaled DA", out_dir, file_identifier, "affine"
    )

    # 두 MAE 결과를 하나의 플롯에 시각화
    if matched_labels or affine_labels:
        fig_mae, ax_mae = plt.subplots(figsize=(18, 7))
        width = 0.35  # the width of the bars

        # Ensure labels are consistent for plotting
        all_labels = sorted(list(set(matched_labels + affine_labels)))
        x = np.arange(len(all_labels))

        matched_mae_dict = dict(zip(matched_labels, matched_mae))
        affine_mae_dict = dict(zip(affine_labels, affine_mae))

        mae_matched_plot = [matched_mae_dict.get(label, 0) for label in all_labels]
        mae_affine_plot = [affine_mae_dict.get(label, 0) for label in all_labels]

        rects1 = ax_mae.bar(x - width/2, mae_matched_plot, width, label='Histogram Matched DA MAE', color='skyblue')
        rects2 = ax_mae.bar(x + width/2, mae_affine_plot, width, label='Affine Scaled DA MAE', color='salmon')

        ax_mae.set_ylabel('Mean Absolute Error (m)')
        ax_mae.set_xlabel('Distance Bin (m)')
        ax_mae.set_title(f'MAE Comparison by Distance Bins (File: {file_identifier})')
        ax_mae.set_xticks(x)
        ax_mae.set_xticklabels(all_labels, rotation=45, ha="right", fontsize=8)
        ax_mae.legend()
        ax_mae.grid(axis='y', linestyle='--', alpha=0.7)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        output_path_mae = out_dir / f"fig_mae_comparison_{file_identifier}.png"
        fig_mae.savefig(output_path_mae, dpi=120)
        print(f"[SAVE] MAE 비교 분석 결과 저장: {output_path_mae}")

    plt.show()


if __name__ == '__main__':
    analyze_and_visualize_distributions()
