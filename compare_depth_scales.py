import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import os

# --- 사용자 설정 ---
# 파일 경로 설정
depth_anything_model_output_path = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/output/single_image_test.raw.npy")
existing_depth_map_folder = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/ncdb-cls-sample/synced_data/depth_maps")
binary_mask_path = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/ncdb-cls-sample/synced_data/binary_mask.png")

# 사용자로부터 설정 값 입력받기
file_identifier = input("비교할 기존 깊이 맵의 10자리 파일명을 입력하세요 (예: 0000000931): ")
if not file_identifier.isdigit() or len(file_identifier) != 10:
    print("오류: 10자리 숫자로 된 파일명을 입력해야 합니다.")
    exit()

while True:
    try:
        threshold_input = input("분포 매칭에 사용할 거리 임계값(m)을 입력하세요 (기본값: 8.0): ")
        if not threshold_input:
            matching_threshold_m = 8.0
            break
        matching_threshold_m = float(threshold_input)
        if matching_threshold_m > 0:
            break
        else:
            print("오류: 0보다 큰 값을 입력해야 합니다.")
    except ValueError:
        print("오류: 유효한 숫자를 입력하세요.")

print(f"\n--- 설정 값 ---")
print(f"파일 식별자: {file_identifier}")
print(f"매칭 임계값: {matching_threshold_m} m")
print("-----------------\n")

existing_depth_map_path = existing_depth_map_folder / f"{file_identifier}.png"
if not existing_depth_map_path.exists():
    print(f"오류: 파일을 찾을 수 없습니다: {existing_depth_map_path}")
    exit()

# --- 데이터 로드 ---
try:
    depth_anything_model_output = np.load(str(depth_anything_model_output_path))
except FileNotFoundError:
    print(f"Error: Could not find Depth Anything V2 model output at {depth_anything_model_output_path}")
    exit()

depth_anything_model_output = np.max(depth_anything_model_output) - depth_anything_model_output
print("[INFO] Inverted Depth Anything model output to align with depth map characteristics.")

existing_depth = cv.imread(str(existing_depth_map_path), cv.IMREAD_UNCHANGED)
if existing_depth is None:
    print(f"Error: Could not load existing depth map from {existing_depth_map_path}")
    exit()

# --- 데이터 전처리 및 마스킹 ---
if depth_anything_model_output.shape != existing_depth.shape:
    print(f"Warning: Resizing existing depth map from {existing_depth.shape} to {depth_anything_model_output.shape}")
    existing_depth = cv.resize(existing_depth, (depth_anything_model_output.shape[1], depth_anything_model_output.shape[0]), interpolation=cv.INTER_NEAREST)

depth_anything_model_output = depth_anything_model_output.astype(np.float32)

try:
    binary_mask = cv.imread(str(binary_mask_path), cv.IMREAD_GRAYSCALE)
    if binary_mask is None: raise FileNotFoundError
except Exception:
    print(f"오류: 바이너리 마스크를 불러올 수 없습니다: {binary_mask_path}")
    exit()

if binary_mask.shape != depth_anything_model_output.shape:
    print(f"오류: 깊이 맵과 마스크의 크기가 다릅니다. (DA: {depth_anything_model_output.shape}, Mask: {binary_mask.shape})")
    exit()

binary_mask_normalized = (binary_mask > 0).astype(np.uint8)
depth_anything_model_output = np.where(binary_mask_normalized == 1, depth_anything_model_output, 0)
print("[INFO] Applied binary mask to Depth Anything model output.")

existing_depth_meters = existing_depth.astype(np.float32) / 256.0
existing_depth_meters = np.where(binary_mask_normalized == 1, existing_depth_meters, 0)
print("[INFO] Applied binary mask to existing depth map.")

# --- 통계 및 시각화 (Figure 1) ---
valid_da_output = depth_anything_model_output[depth_anything_model_output > 0]
valid_existing_depth_meters = existing_depth_meters[existing_depth_meters > 0.1]

if valid_da_output.size == 0 or valid_existing_depth_meters.size == 0:
    print("오류: 유효한 데이터가 없습니다. 마스크 영역을 확인하세요.")
    exit()

vmin_da, vmax_da = (np.percentile(valid_da_output, [0.5, 99.5]))
vmin_existing, vmax_existing = (np.percentile(valid_existing_depth_meters, [0.5, 99.5]))

out_dir = Path(f"output/0401_calib_data_scale_compare_{file_identifier}_{matching_threshold_m}m")
out_dir.mkdir(parents=True, exist_ok=True)

fig1, axes1 = plt.subplots(2, 2, figsize=(16, 14))
fig1.suptitle('Masked Original Data Comparison', fontsize=18)
im1 = axes1[0, 0].imshow(depth_anything_model_output, cmap='magma', vmin=vmin_da, vmax=vmax_da)
axes1[0, 0].set_title('Masked Depth Anything V2 Output (Raw)')
fig1.colorbar(im1, ax=axes1[0, 0], label='Raw Model Output')
im2 = axes1[0, 1].imshow(existing_depth_meters, cmap='magma', vmin=vmin_existing, vmax=vmax_existing)
axes1[0, 1].set_title('Masked Existing Depth Map (Meters)')
fig1.colorbar(im2, ax=axes1[0, 1], label='Depth (m)')
axes1[1, 0].hist(valid_da_output.flatten(), bins=100, color='skyblue', edgecolor='black', range=(vmin_da, vmax_da))
axes1[1, 0].set_title('Histogram: Masked DA Output')
axes1[1, 0].set_xlabel('Raw Model Output Value')
axes1[1, 0].grid(True, alpha=0.5)
axes1[1, 1].hist(valid_existing_depth_meters.flatten(), bins=100, color='lightcoral', edgecolor='black', range=(vmin_existing, vmax_existing))
axes1[1, 1].set_title('Histogram: Masked Existing Depth Map')
axes1[1, 1].set_xlabel('Depth (Meters)')
axes1[1, 1].grid(True, alpha=0.5)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
fig1_path = out_dir / "fig1_original_comparison.png"
fig1.savefig(fig1_path, dpi=120)
print(f"[SAVE] Figure 1 saved: {fig1_path}")

# --- 선형 스케일링 및 분석 (Figure 2, 3) ---
print(f"\n--- Linear Scaling (based on < {matching_threshold_m}m) ---")
scaling_mask = (existing_depth_meters > 0.1) & (existing_depth_meters < matching_threshold_m) & (depth_anything_model_output > 0)

if np.sum(scaling_mask) > 10:
    da_values_for_scaling = depth_anything_model_output[scaling_mask]
    existing_values_for_scaling = existing_depth_meters[scaling_mask]
    mean_da = np.mean(da_values_for_scaling)
    mean_existing = np.mean(existing_values_for_scaling)
    
    if mean_da > 0:
        scale_factor = mean_existing / mean_da
        print(f"Calculated Scale Factor: {scale_factor:.6f}")
        scaled_depth_anything_output = np.where(binary_mask_normalized == 1, depth_anything_model_output * scale_factor, 0)
        valid_scaled_output = scaled_depth_anything_output[scaled_depth_anything_output > 0]

        # Figure 2: 선형 스케일링 진단
        fig2, axes2 = plt.subplots(2, 3, figsize=(20, 12))
        fig2.suptitle('Scale Adjustment Diagnostics (Masked)', fontsize=20)
        vmin_final = min(np.percentile(valid_existing_depth_meters, 0.5), np.percentile(valid_scaled_output, 0.5))
        vmax_final = max(np.percentile(valid_existing_depth_meters, 99.5), np.percentile(valid_scaled_output, 99.5))

        im_a = axes2[0,0].imshow(existing_depth_meters, cmap='magma', vmin=vmin_final, vmax=vmax_final)
        axes2[0,0].set_title('Existing Depth (GT)')
        fig2.colorbar(im_a, ax=axes2[0,0], fraction=0.046, pad=0.02)
        im_b = axes2[0,1].imshow(scaled_depth_anything_output, cmap='magma', vmin=vmin_final, vmax=vmax_final)
        axes2[0,1].set_title('Scaled Depth Anything')
        fig2.colorbar(im_b, ax=axes2[0,1], fraction=0.046, pad=0.02)
        diff = scaled_depth_anything_output - existing_depth_meters
        diff_masked = diff[(binary_mask_normalized == 1) & (existing_depth_meters > 0.1)]
        diff_lim = max(np.percentile(np.abs(diff_masked), 95), 1e-3) if diff_masked.size else 0.1
        cmap_diff = plt.get_cmap('coolwarm').copy()
        cmap_diff.set_bad('lightgray')
        diff_display = np.clip(diff, -diff_lim, diff_lim)
        diff_display[scaled_depth_anything_output == 0] = np.nan
        im_c = axes2[0,2].imshow(diff_display, cmap=cmap_diff, vmin=-diff_lim, vmax=diff_lim)
        axes2[0,2].set_title(f'Diff (Scaled - GT) ±{diff_lim:.2f} m')
        fig2.colorbar(im_c, ax=axes2[0,2], fraction=0.046, pad=0.02)
        with np.errstate(divide='ignore', invalid='ignore'): rel = diff / existing_depth_meters
        rel_masked = rel[(binary_mask_normalized == 1) & (existing_depth_meters > 0.1) & np.isfinite(rel)]
        rel_lim = np.clip(np.percentile(np.abs(rel_masked), 95), 0.05, 0.5) if rel_masked.size else 0.5
        cmap_rel = plt.get_cmap('seismic').copy()
        cmap_rel.set_bad('lightgray')
        rel_display = np.clip(rel, -rel_lim, rel_lim)
        rel_display[~np.isfinite(rel_display)] = np.nan
        im_d = axes2[1,0].imshow(rel_display, cmap=cmap_rel, vmin=-rel_lim, vmax=rel_lim)
        axes2[1,0].set_title(f'Relative Diff ((S-G)/G) ±{rel_lim*100:.1f}%')
        fig2.colorbar(im_d, ax=axes2[1,0], fraction=0.046, pad=0.02)
        valid_mask = (binary_mask_normalized==1) & (existing_depth_meters > 0.1) & (scaled_depth_anything_output > 0)
        x_gt = existing_depth_meters[valid_mask]
        y_sc = scaled_depth_anything_output[valid_mask]
        axes2[1,1].plot([vmin_final, vmax_final],[vmin_final, vmax_final],'k--',lw=1,label='y=x')
        if x_gt.size > 20000:
            hb = axes2[1,1].hexbin(x_gt, y_sc, gridsize=60, bins='log', cmap='viridis', extent=[vmin_final, vmax_final, vmin_final, vmax_final])
            fig2.colorbar(hb, ax=axes2[1,1], fraction=0.046, pad=0.02, label='log(count)')
        else:
            axes2[1,1].scatter(x_gt, y_sc, s=6, c='tab:blue', alpha=0.4, edgecolors='none')
        axes2[1,1].set_xlim(vmin_final, vmax_final); axes2[1,1].set_ylim(vmin_final, vmax_final)
        axes2[1,1].set_xlabel('GT Depth (m)'); axes2[1,1].set_ylabel('Scaled Depth (m)')
        axes2[1,1].set_title('GT vs Scaled'); axes2[1,1].legend()
        if x_gt.size > 100:
            qs = np.linspace(0.01, 0.99, 200)
            gt_q = np.quantile(x_gt, qs); sc_q = np.quantile(y_sc, qs)
            axes2[1,2].plot(gt_q, sc_q, label='Quantiles')
            axes2[1,2].plot([gt_q.min(), gt_q.max()],[gt_q.min(), gt_q.max()],'k--',lw=1,label='y=x')
            axes2[1,2].set_xlabel('GT Quantiles (m)'); axes2[1,2].set_ylabel('Scaled Quantiles (m)')
            axes2[1,2].set_title('Q-Q Plot'); axes2[1,2].grid(alpha=0.3); axes2[1,2].legend()
        else:
            axes2[1,2].text(0.5,0.5,'Not enough samples',ha='center',va='center'); axes2[1,2].set_axis_off()
        plt.tight_layout(rect=[0,0.03,1,0.95])
        fig2_path = out_dir / "fig2_linear_scaling_diagnostics.png"
        fig2.savefig(fig2_path, dpi=120)
        print(f"[SAVE] Figure 2 saved: {fig2_path}")

        # Figure 3: 선형 스케일링 분포 분석
        fig3 = plt.figure(figsize=(18,10))
        fig3.suptitle('Distribution Analysis after Linear Scaling (Masked)', fontsize=20)
        bins = np.linspace(vmin_final, vmax_final, 100)
        if bins.size < 2: bins = 50
        ax_h1 = fig3.add_subplot(2,2,1); ax_h1.hist(x_gt, bins=bins, alpha=0.5, label='GT', color='lightcoral'); ax_h1.hist(y_sc, bins=bins, alpha=0.5, label='Scaled', color='mediumpurple'); ax_h1.set_title('Histogram (Linear)'); ax_h1.legend(); ax_h1.grid(alpha=0.3)
        ax_h2 = fig3.add_subplot(2,2,2); ax_h2.hist(x_gt, bins=bins, alpha=0.5, label='GT', color='lightcoral', log=True); ax_h2.hist(y_sc, bins=bins, alpha=0.5, label='Scaled', color='mediumpurple', log=True); ax_h2.set_title('Histogram (Log)'); ax_h2.legend(); ax_h2.grid(alpha=0.3)
        half_thresh = matching_threshold_m / 2
        ranges = [(0, half_thresh), (half_thresh, matching_threshold_m)]
        for i,(lo,hi) in enumerate(ranges, start=3):
            ax = fig3.add_subplot(2,2,i)
            rmask = (x_gt>=lo) & (x_gt<hi)
            if np.sum(rmask):
                ax.hist(x_gt[rmask], bins=30, alpha=0.5, label='GT', color='lightcoral'); ax.hist(y_sc[rmask], bins=30, alpha=0.5, label='Scaled', color='mediumpurple')
                ax.set_title(f'{lo:.1f}-{hi:.1f} m'); ax.legend(); ax.grid(alpha=0.3)
            else:
                ax.text(0.5,0.5,'No samples',ha='center',va='center'); ax.set_axis_off()
        plt.tight_layout(rect=[0,0.03,1,0.95])
        fig3_path = out_dir / "fig3_linear_scaling_distribution.png"
        fig3.savefig(fig3_path, dpi=120)
        print(f"[SAVE] Figure 3 saved: {fig3_path}")
else:
    print(f"[WARN] Linear scaling region (< {matching_threshold_m}m) has too few points. Skipping Figure 2 & 3.")

# --- 히스토그램 매칭 및 분석 (Figure 4, 5, 6) ---
print(f"\n--- Histogram Matching (based on < {matching_threshold_m}m) ---")
if np.sum(scaling_mask) > 10:
    da_values_for_matching = depth_anything_model_output[scaling_mask]
    existing_values_for_matching = existing_depth_meters[scaling_mask]
    sorted_source = np.sort(da_values_for_matching)
    sorted_template = np.sort(existing_values_for_matching)
    matched_values = np.interp(valid_da_output, sorted_source, sorted_template)
    matched_depth_output = np.zeros_like(depth_anything_model_output)
    matched_depth_output[depth_anything_model_output > 0] = matched_values
    print("Histogram matching applied.")

    # Figure 4
    fig4, axes4 = plt.subplots(2, 2, figsize=(16, 14))
    fig4.suptitle(f'Result After Histogram Matching (Learned from < {matching_threshold_m}m)', fontsize=18)
    im_gt = axes4[0, 0].imshow(existing_depth_meters, cmap='magma', vmin=vmin_existing, vmax=vmax_existing)
    axes4[0, 0].set_title('Masked Existing Depth (GT)'); fig4.colorbar(im_gt, ax=axes4[0, 0], label='Depth (m)')
    im_matched = axes4[0, 1].imshow(matched_depth_output, cmap='magma', vmin=vmin_existing, vmax=vmax_existing)
    axes4[0, 1].set_title('Histogram Matched DA Output'); fig4.colorbar(im_matched, ax=axes4[0, 1], label='Depth (m)')
    bins_all = np.linspace(vmin_existing, vmax_existing, 100)
    axes4[1, 0].hist(valid_existing_depth_meters, bins=bins_all, color='lightcoral', alpha=0.6, label='GT', histtype='stepfilled')
    axes4[1, 0].hist(matched_depth_output[matched_depth_output > 0], bins=bins_all, color='mediumpurple', label='Matched DA', histtype='step', linewidth=1.5)
    axes4[1, 0].set_title('Histogram of All Valid Pixels'); axes4[1, 0].legend(); axes4[1, 0].grid(True, alpha=0.3)
    bins_under = np.linspace(0, matching_threshold_m, 100)
    gt_under = existing_depth_meters[scaling_mask]
    matched_under = matched_depth_output[scaling_mask]
    axes4[1, 1].hist(gt_under, bins=bins_under, color='lightcoral', alpha=0.6, label=f'GT (< {matching_threshold_m}m)', histtype='stepfilled')
    axes4[1, 1].hist(matched_under, bins=bins_under, color='mediumpurple', label=f'Matched DA (< {matching_threshold_m}m)', histtype='step', linewidth=1.5)
    axes4[1, 1].set_title(f'Histogram of Pixels in < {matching_threshold_m}m Region'); axes4[1, 1].legend(); axes4[1, 1].grid(True, alpha=0.3)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig4_path = out_dir / "fig4_hist_matched.png"
    fig4.savefig(fig4_path, dpi=120)
    print(f"[SAVE] Figure 4 saved: {fig4_path}")

    # Figure 5
    verification_mask = (existing_depth_meters >= matching_threshold_m) & (matched_depth_output > 0)
    if np.sum(verification_mask) > 10:
        gt_over = existing_depth_meters[verification_mask]
        matched_over = matched_depth_output[verification_mask]
        fig5, axes5 = plt.subplots(2, 2, figsize=(16, 14))
        fig5.suptitle(f'Generalization Check for Distances >= {matching_threshold_m}m', fontsize=18)
        vmin_verify = np.percentile(gt_over, 1); vmax_verify = np.percentile(gt_over, 99)
        if np.isclose(vmin_verify, vmax_verify): vmax_verify += 1e-3
        gt_display_over = np.where(existing_depth_meters >= matching_threshold_m, existing_depth_meters, 0)
        im_gt_v = axes5[0, 0].imshow(gt_display_over, cmap='magma', vmin=vmin_verify, vmax=vmax_verify)
        axes5[0, 0].set_title(f'GT Depth Map (>= {matching_threshold_m}m)'); fig5.colorbar(im_gt_v, ax=axes5[0, 0], label='Depth (m)')
        matched_display_over = np.where(existing_depth_meters >= matching_threshold_m, matched_depth_output, 0)
        im_matched_v = axes5[0, 1].imshow(matched_display_over, cmap='magma', vmin=vmin_verify, vmax=vmax_verify)
        axes5[0, 1].set_title(f'Matched DA Map (>= {matching_threshold_m}m)'); fig5.colorbar(im_matched_v, ax=axes5[0, 1], label='Depth (m)')
        bins_verify = np.linspace(vmin_verify, vmax_verify, 100)
        axes5[1, 0].hist(gt_over, bins=bins_verify, color='lightcoral', alpha=0.6, label=f'GT (>= {matching_threshold_m}m)', histtype='stepfilled')
        axes5[1, 0].hist(matched_over, bins=bins_verify, color='mediumpurple', label=f'Matched DA (>= {matching_threshold_m}m)', histtype='step', linewidth=1.5)
        axes5[1, 0].set_title(f'Histogram Comparison for >= {matching_threshold_m}m Region'); axes5[1, 0].legend(); axes5[1, 0].grid(True, alpha=0.3)
        qs_verify = np.linspace(0, 1, 100)
        gt_q_verify = np.quantile(gt_over, qs_verify); matched_q_verify = np.quantile(matched_over, qs_verify)
        axes5[1, 1].plot(gt_q_verify, matched_q_verify, 'o-', label='Q-Q', color='darkgreen', alpha=0.7)
        min_val, max_val = min(gt_q_verify.min(), matched_q_verify.min()), max(gt_q_verify.max(), matched_q_verify.max())
        axes5[1, 1].plot([min_val, max_val], [min_val, max_val], 'k--', label='y=x')
        axes5[1, 1].set_title(f'Q-Q Plot for >= {matching_threshold_m}m Region'); axes5[1, 1].legend(); axes5[1, 1].grid(True, alpha=0.3)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig5_path = out_dir / "fig5_verification.png"
        fig5.savefig(fig5_path, dpi=120)
        print(f"[SAVE] Figure 5 saved: {fig5_path}")
    else:
        print(f"[INFO] Not enough data points (> 10) in the >= {matching_threshold_m}m region to create verification plot.")
    
    # Figure 6: 거리별 오차 분석
    print("\n--- Error Analysis by Distance (Figure 6) ---")
    valid_pixels_mask = (existing_depth_meters > 0.1) & (matched_depth_output > 0)
    gt_valid = existing_depth_meters[valid_pixels_mask]
    matched_valid = matched_depth_output[valid_pixels_mask]
    
    max_dist = np.max(gt_valid)
    bins = np.arange(0, max_dist + 2, 2)
    
    bin_labels = []
    mae_values = []
    mre_values = []
    
    for i in range(len(bins) - 1):
        lower_bound, upper_bound = bins[i], bins[i+1]
        bin_mask = (gt_valid >= lower_bound) & (gt_valid < upper_bound)
        
        if np.sum(bin_mask) > 10:
            gt_bin = gt_valid[bin_mask]
            matched_bin = matched_valid[bin_mask]
            
            abs_error = np.abs(matched_bin - gt_bin)
            rel_error = abs_error / gt_bin
            
            mae = np.mean(abs_error)
            mre = np.mean(rel_error) * 100  # percentage
            
            bin_labels.append(f'{lower_bound}-{upper_bound}m')
            mae_values.append(mae)
            mre_values.append(mre)

    if bin_labels:
        fig6, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 7))
        fig6.suptitle('Error Analysis by Distance Bins', fontsize=12)
        
        ax1.bar(bin_labels, mae_values, color='skyblue')
        ax1.tick_params(axis='x', labelsize=8, labelrotation=45)
        ax1.set_title('Mean Absolute Error (MAE) by Distance')
        ax1.set_xlabel('Distance Bin (m)')
        ax1.set_ylabel('Mean Absolute Error (m)')
        ax1.grid(axis='y', linestyle='--', alpha=0.7)
        
        ax2.bar(bin_labels, mre_values, color='salmon')
        ax2.tick_params(axis='x', labelsize=8, labelrotation=45)
        ax2.set_title('Mean Relative Error (MRE) by Distance')
        ax2.set_xlabel('Distance Bin (m)')
        ax2.set_ylabel('Mean Relative Error (%)')
        ax2.grid(axis='y', linestyle='--', alpha=0.7)
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig6_path = out_dir / "fig6_error_by_distance.png"
        fig6.savefig(fig6_path, dpi=120)
        print(f"[SAVE] Figure 6 saved: {fig6_path}")
    else:
        print("[INFO] Not enough data to generate error analysis plot (Figure 6).")

    # # Figure 7: Scaled DA vs GT Comparison (Temporarily Disabled)
    # if 'scaled_depth_anything_output' in locals():
    #     print("\n--- Figure 7 Data Range ---")
    #     print(f"vmin_final (used for color range): {vmin_final}")
    #     print(f"vmax_final (used for color range): {vmax_final}")

    #     valid_gt_fig7 = existing_depth_meters[(existing_depth_meters > 0.1) & (scaled_depth_anything_output > 0)]
    #     valid_scaled_fig7 = scaled_depth_anything_output[(existing_depth_meters > 0.1) & (scaled_depth_anything_output > 0)]

    #     if valid_gt_fig7.size > 0 and valid_scaled_fig7.size > 0:
    #         print("Ground Truth Percentiles (for valid overlapping area):")
    #         print(f"  min: {np.min(valid_gt_fig7):.4f}")
    #         print(f"  1%: {np.percentile(valid_gt_fig7, 1):.4f}")
    #         print(f"  50%: {np.percentile(valid_gt_fig7, 50):.4f}")
    #         print(f"  99%: {np.percentile(valid_gt_fig7, 99):.4f}")
    #         print(f"  max: {np.max(valid_gt_fig7):.4f}")

    #         print("Scaled DA Percentiles (for valid overlapping area):")
    #         print(f"  min: {np.min(valid_scaled_fig7):.4f}")
    #         print(f"  1%: {np.percentile(valid_scaled_fig7, 1):.4f}")
    #         print(f"  50%: {np.percentile(valid_scaled_fig7, 50):.4f}")
    #         print(f"  99%: {np.percentile(valid_scaled_fig7, 99):.4f}")
    #         print(f"  max: {np.max(valid_scaled_fig7):.4f}")
    #     print("---------------------------\n")
        
    #     fig7, axes7 = plt.subplots(1, 2, figsize=(16, 8))
    #     fig7.suptitle('Linearly Scaled DA vs. Ground Truth', fontsize=16)
        
    #     # Use the same vmin/vmax from Figure 2 for consistency
    #     im_gt_fig7 = axes7[0].imshow(existing_depth_meters, cmap='magma_r', vmin=vmin_final, vmax=vmax_final)
    #     axes7[0].set_title('Ground Truth Depth Map')
    #     fig7.colorbar(im_gt_fig7, ax=axes7[0], label='Depth (m)')
        
    #     im_scaled_fig7 = axes7[1].imshow(scaled_depth_anything_output, cmap='magma_r', vmin=vmin_final, vmax=vmax_final)
    #     axes7[1].set_title('Linearly Scaled Depth Anything')
    #     fig7.colorbar(im_scaled_fig7, ax=axes7[1], label='Depth (m)')
        
    #     plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    #     fig7_path = out_dir / "fig7_scaled_vs_gt_comparison.png"
    #     fig7.savefig(fig7_path, dpi=120)
    #     print(f"[SAVE] Figure 7 saved: {fig7_path}")

    # Figure 8: Comprehensive Comparison
    if 'matched_depth_output' in locals() and 'scaled_depth_anything_output' in locals():
        fig8, axes8 = plt.subplots(1, 3, figsize=(24, 8))
        fig8.suptitle('Comprehensive Comparison: GT vs. Scaled vs. Matched', fontsize=16)
        
        # Calculate individual ranges for display in titles
        valid_scaled_output_fig8 = scaled_depth_anything_output[scaled_depth_anything_output > 0]
        valid_matched_output_fig8 = matched_depth_output[matched_depth_output > 0]

        vmin_gt_fig8, vmax_gt_fig8 = np.percentile(existing_depth_meters[existing_depth_meters > 0.1], [0.5, 99.5])
        vmin_scaled_fig8, vmax_scaled_fig8 = (np.percentile(valid_scaled_output_fig8, [0.5, 99.5]) if valid_scaled_output_fig8.size > 0 else (0, 1))
        vmin_matched_fig8, vmax_matched_fig8 = (np.percentile(valid_matched_output_fig8, [0.5, 99.5]) if valid_matched_output_fig8.size > 0 else (0, 1))

        # Calculate overall min/max for consistent color scaling across all subplots
        # Use the minimum of all individual vmins and maximum of all individual vmaxs
        vmin_overall_fig8 = min(vmin_gt_fig8, vmin_scaled_fig8, vmin_matched_fig8)
        vmax_overall_fig8 = max(vmax_gt_fig8, vmax_scaled_fig8, vmax_matched_fig8)

        # 1. Ground Truth
        im_gt_fig8 = axes8[0].imshow(existing_depth_meters, cmap='magma', vmin=vmin_overall_fig8, vmax=vmax_overall_fig8)
        axes8[0].set_title(f'Ground Truth\nRange: [{vmin_gt_fig8:.2f}, {vmax_gt_fig8:.2f}] m')
        fig8.colorbar(im_gt_fig8, ax=axes8[0], label='Depth (m)', fraction=0.046, pad=0.04)
        
        # 2. Linearly Scaled DA
        im_scaled_fig8 = axes8[1].imshow(scaled_depth_anything_output, cmap='magma', vmin=vmin_overall_fig8, vmax=vmax_overall_fig8)
        axes8[1].set_title(f'Linearly Scaled DA\nRange: [{vmin_scaled_fig8:.2f}, {vmax_scaled_fig8:.2f}] m')
        fig8.colorbar(im_scaled_fig8, ax=axes8[1], label='Depth (m)', fraction=0.046, pad=0.04)

        # 3. Histogram Matched DA
        im_matched_fig8 = axes8[2].imshow(matched_depth_output, cmap='magma', vmin=vmin_overall_fig8, vmax=vmax_overall_fig8)
        axes8[2].set_title(f'Histogram Matched DA\nRange: [{vmin_matched_fig8:.2f}, {vmax_matched_fig8:.2f}] m')
        fig8.colorbar(im_matched_fig8, ax=axes8[2], label='Depth (m)', fraction=0.046, pad=0.04)

        print(f"[INFO] Figure 8 - Ground Truth Range: [{vmin_gt_fig8:.2f}, {vmax_gt_fig8:.2f}] m")
        print(f"[INFO] Figure 8 - Linearly Scaled DA Range: [{vmin_scaled_fig8:.2f}, {vmax_scaled_fig8:.2f}] m")
        print(f"[INFO] Figure 8 - Histogram Matched DA Range: [{vmin_matched_fig8:.2f}, {vmax_matched_fig8:.2f}] m")
        print(f"[INFO] Figure 8 - Overall Display Range (vmin, vmax): [{vmin_overall_fig8:.2f}, {vmax_overall_fig8:.2f}] m")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig8_path = out_dir / "fig8_comprehensive_comparison.png"
        fig8.savefig(fig8_path, dpi=120)
        print(f"[SAVE] Figure 8 saved: {fig8_path}")

else:
    print(f"[WARN] Matching region (< {matching_threshold_m}m) has too few points. Skipping Figure 4, 5, & 6.")

if 'plt' in locals() and 'fig1' in locals():
    backend = plt.get_backend()
    if 'agg' not in backend.lower():
        print("[INFO] Showing figures...")
        plt.show()
