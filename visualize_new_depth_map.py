import os
import numpy as np
import cv2
import argparse
from pathlib import Path
import sys

# 전역 변수 초기화
original_depth_map = None  # 원본 깊이 맵 (미터 단위)
synthetic_depth_map = None  # 합성 깊이 맵 (미터 단위)
original_color_mapped = None  # 원본 컬러맵 이미지
synthetic_color_mapped = None  # 합성 컬러맵 이미지
current_display_mode = 'side_by_side'  # 'side_by_side', 'original', 'synthetic', 'difference'

zoom_level = 1.0
pan_x = 0.0
pan_y = 0.0
drag_start_x = 0
drag_start_y = 0
is_dragging = False

# 고정된 창 크기
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080

def load_depth_map_png(file_path):
    """
    16비트 그레이스케일 PNG 깊이 맵을 로드하고 미터 단위로 변환합니다.
    """
    try:
        img = cv2.imread(str(file_path), cv2.IMREAD_UNCHANGED)
        if img is None:
            print(f"Error: Could not load image from {file_path}", file=sys.stderr)
            return None
        
        if img.dtype != np.uint16:
            print(f"Warning: {file_path}는 16비트 이미지가 아닐 수 있습니다. 현재 dtype: {img.dtype}", file=sys.stderr)
            img = img.astype(np.uint16)
        
        depth_map_pixels = np.array(img, dtype=np.float32)
        depth_map_meters = depth_map_pixels / 256.0
        return depth_map_meters
    except Exception as e:
        print(f"PNG 깊이 맵 로드 중 오류 발생 ({file_path}): {e}", file=sys.stderr)
        return None

def create_colorized_depth_map(depth_map, colormap=cv2.COLORMAP_MAGMA, white_background=True):
    """
    깊이 맵을 컬러맵으로 변환합니다.
    """
    if depth_map is None:
        return None
    
    # 유효한 깊이 값만 고려하여 정규화
    valid_mask = depth_map > 0
    if not np.any(valid_mask):
        return np.ones((*depth_map.shape, 3), dtype=np.uint8) * 255
    
    # percentile 기반 범위 설정으로 더 나은 시각화
    vmin, vmax = np.percentile(depth_map[valid_mask], [1, 99])
    
    # 정규화 (0-255 범위)
    normalized_depth = np.zeros_like(depth_map, dtype=np.float32)
    valid_range = vmax - vmin
    if valid_range > 1e-6:
        normalized_depth[valid_mask] = np.clip(
            (depth_map[valid_mask] - vmin) / valid_range * 255, 0, 255
        )
    
    normalized_depth = normalized_depth.astype(np.uint8)
    
    # 컬러맵 적용
    color_mapped = cv2.applyColorMap(normalized_depth, colormap)
    
    # 깊이 값이 0인 부분을 흰색 또는 검은색 배경으로 처리
    if white_background:
        color_mapped[~valid_mask] = [255, 255, 255]  # BGR: 흰색
    else:
        color_mapped[~valid_mask] = [0, 0, 0]  # BGR: 검은색
    
    return color_mapped

def create_difference_map_vectorized(original, synthetic):
    """
    두 깊이 맵의 차이를 시각화합니다 (벡터화된 버전).
    """
    if original is None or synthetic is None:
        return None
    
    # 두 맵 모두 유효한 픽셀만 비교
    valid_mask = (original > 0) & (synthetic > 0)
    if not np.any(valid_mask):
        return np.ones((*original.shape, 3), dtype=np.uint8) * 128
    
    # 차이 계산
    difference = np.zeros_like(original)
    difference[valid_mask] = synthetic[valid_mask] - original[valid_mask]
    
    # 차이를 색상으로 매핑 (-max_diff ~ +max_diff)
    max_diff = np.percentile(np.abs(difference[valid_mask]), 95)
    if max_diff < 1e-6:
        max_diff = 1.0
    
    # 차이를 -1~1 범위로 정규화
    normalized_diff = np.clip(difference / max_diff, -1, 1)
    
    # RGB 채널 생성
    height, width = original.shape
    color_mapped_diff = np.zeros((height, width, 3), dtype=np.uint8)
    
    # 파랑 영역 (val < 0): synthetic < original
    blue_mask = (normalized_diff < 0) & valid_mask
    blue_intensity = np.abs(normalized_diff[blue_mask])
    color_mapped_diff[blue_mask, 0] = 255  # B
    color_mapped_diff[blue_mask, 1] = (255 * (1 - blue_intensity)).astype(np.uint8)  # G
    color_mapped_diff[blue_mask, 2] = (255 * (1 - blue_intensity)).astype(np.uint8)  # R
    
    # 빨강 영역 (val > 0): synthetic > original
    red_mask = (normalized_diff > 0) & valid_mask
    red_intensity = normalized_diff[red_mask]
    color_mapped_diff[red_mask, 0] = (255 * (1 - red_intensity)).astype(np.uint8)  # B
    color_mapped_diff[red_mask, 1] = (255 * (1 - red_intensity)).astype(np.uint8)  # G
    color_mapped_diff[red_mask, 2] = 255  # R
    
    # 차이가 0인 영역 (흰색)
    zero_mask = (normalized_diff == 0) & valid_mask
    color_mapped_diff[zero_mask] = [255, 255, 255]  # BGR: 흰색
    
    # 무효 픽셀은 회색으로
    color_mapped_diff[~valid_mask] = [128, 128, 128]  # BGR: 회색
    
    return color_mapped_diff

# 기존 create_difference_map 함수를 벡터화된 버전으로 교체
def create_difference_map(original, synthetic):
    """
    두 깊이 맵의 차이를 시각화합니다.
    """
    return create_difference_map_vectorized(original, synthetic)

def update_display_image():
    """
    현재 표시 모드와 줌/팬 상태에 따라 화면 이미지를 업데이트합니다.
    """
    global original_color_mapped, synthetic_color_mapped, current_display_mode
    global zoom_level, pan_x, pan_y, WINDOW_WIDTH, WINDOW_HEIGHT
    
    if original_color_mapped is None and synthetic_color_mapped is None:
        return
    
    # 표시할 이미지 선택
    if current_display_mode == 'side_by_side':
        if original_color_mapped is not None and synthetic_color_mapped is not None:
            # [FIX] 두 이미지를 동일한 크기로 리사이즈
            h_orig, w_orig = original_color_mapped.shape[:2]
            h_synth, w_synth = synthetic_color_mapped.shape[:2]
            
            # 공통 높이를 더 작은 값으로 설정하거나, 더 큰 값으로 통일
            target_height = min(h_orig, h_synth)  # 작은 높이로 통일
            # target_height = max(h_orig, h_synth)  # 큰 높이로 통일하려면 이 줄 사용
            
            # 비율을 유지하면서 리사이즈
            scale_orig = target_height / h_orig
            scale_synth = target_height / h_synth
            
            new_w_orig = int(w_orig * scale_orig)
            new_w_synth = int(w_synth * scale_synth)
            
            # 리사이즈 수행
            resized_orig = cv2.resize(original_color_mapped, (new_w_orig, target_height), 
                                    interpolation=cv2.INTER_LINEAR)
            resized_synth = cv2.resize(synthetic_color_mapped, (new_w_synth, target_height), 
                                     interpolation=cv2.INTER_LINEAR)
            
            # 두 이미지를 나란히 배치
            total_width = new_w_orig + new_w_synth
            combined_image = np.zeros((target_height, total_width, 3), dtype=np.uint8) + 255
            combined_image[:, :new_w_orig] = resized_orig
            combined_image[:, new_w_orig:] = resized_synth
            
            # 중앙 구분선 추가
            cv2.line(combined_image, (new_w_orig, 0), (new_w_orig, target_height-1), (0, 0, 0), 2)
            
            # 크기 정보 텍스트 추가
            text_orig = f"Original: {h_orig}x{w_orig}"
            text_synth = f"Synthetic: {h_synth}x{w_synth}"
            cv2.putText(combined_image, text_orig, (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(combined_image, text_synth, (new_w_orig + 10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            
            display_source = combined_image
        else:
            display_source = original_color_mapped or synthetic_color_mapped
    elif current_display_mode == 'original':
        display_source = original_color_mapped
    elif current_display_mode == 'synthetic':
        display_source = synthetic_color_mapped
    elif current_display_mode == 'difference':
        if original_depth_map is not None and synthetic_depth_map is not None:
            # [FIX] 차이 맵 계산 전에 크기를 맞춤
            h_orig, w_orig = original_depth_map.shape[:2]
            h_synth, w_synth = synthetic_depth_map.shape[:2]
            
            if (h_orig, w_orig) != (h_synth, w_synth):
                # 합성 이미지를 원본 크기에 맞춤
                synthetic_resized = cv2.resize(synthetic_depth_map, (w_orig, h_orig), 
                                             interpolation=cv2.INTER_LINEAR)
                display_source = create_difference_map(original_depth_map, synthetic_resized)
            else:
                display_source = create_difference_map(original_depth_map, synthetic_depth_map)
        else:
            display_source = original_color_mapped or synthetic_color_mapped
    else:
        display_source = original_color_mapped or synthetic_color_mapped
    
    if display_source is None:
        return
    
    h_orig, w_orig = display_source.shape[:2]
    
    # 줌 적용
    new_w, new_h = int(w_orig * zoom_level), int(h_orig * zoom_level)
    zoomed_image = cv2.resize(display_source, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # 팬 범위 제한
    if new_w < WINDOW_WIDTH:
        pan_x = (WINDOW_WIDTH - new_w) / 2
    else:
        pan_x = max(min(pan_x, 0.0), -(new_w - WINDOW_WIDTH))
    
    if new_h < WINDOW_HEIGHT:
        pan_y = (WINDOW_HEIGHT - new_h) / 2
    else:
        pan_y = max(min(pan_y, 0.0), -(new_h - WINDOW_HEIGHT))
    
    # 캔버스 생성
    canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8) + 255
    
    # 복사할 영역 계산
    src_x1 = max(0, -int(pan_x))
    src_y1 = max(0, -int(pan_y))
    src_x2 = min(new_w, src_x1 + WINDOW_WIDTH)
    src_y2 = min(new_h, src_y1 + WINDOW_HEIGHT)
    
    dst_x1 = max(0, int(pan_x))
    dst_y1 = max(0, int(pan_y))
    dst_x2 = min(WINDOW_WIDTH, dst_x1 + (src_x2 - src_x1))
    dst_y2 = min(WINDOW_HEIGHT, dst_y1 + (src_y2 - src_y1))
    
    # 이미지 복사
    canvas[dst_y1:dst_y2, dst_x1:dst_x2] = zoomed_image[src_y1:src_y2, src_x1:src_x2]
    
    # 상태 정보 표시
    info_text = [
        f"Mode: {current_display_mode}",
        f"Zoom: {zoom_level:.2f}x",
        "Keys: [1]Original [2]Synthetic [3]Side-by-side [4]Difference",
        "[Scroll]Zoom [Drag]Pan [Q]uit [S]ave"
    ]
    
    y_offset = 30
    for i, text in enumerate(info_text):
        cv2.putText(canvas, text, (10, y_offset + i * 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
    
    cv2.imshow("Depth Map Comparison", canvas)

def mouse_callback(event, x, y, flags, param):
    """마우스 이벤트 콜백 함수"""
    global zoom_level, pan_x, pan_y, drag_start_x, drag_start_y, is_dragging
    global original_depth_map, synthetic_depth_map, current_display_mode
    
    # 마우스 휠 이벤트 (확대/축소)
    if event == cv2.EVENT_MOUSEWHEEL:
        zoom_factor = 1.1 if flags > 0 else 1 / 1.1
        new_zoom_level = max(0.1, min(10.0, zoom_level * zoom_factor))
        
        if new_zoom_level != zoom_level:
            # 줌 중심을 마우스 위치로 유지
            if current_display_mode == 'side_by_side' and original_depth_map is not None and synthetic_depth_map is not None:
                # side-by-side 모드에서는 원본 이미지 크기 기준
                orig_w = original_depth_map.shape[1]
                orig_x_at_mouse = (x - pan_x) / zoom_level
                orig_y_at_mouse = (y - pan_y) / zoom_level
            else:
                orig_x_at_mouse = (x - pan_x) / zoom_level
                orig_y_at_mouse = (y - pan_y) / zoom_level
            
            zoom_level = new_zoom_level
            pan_x = x - (orig_x_at_mouse * zoom_level)
            pan_y = y - (orig_y_at_mouse * zoom_level)
            
            update_display_image()
    
    # 마우스 드래그 이벤트 (패닝)
    elif event == cv2.EVENT_LBUTTONDOWN:
        is_dragging = True
        drag_start_x = x
        drag_start_y = y
    elif event == cv2.EVENT_MOUSEMOVE and is_dragging:
        dx = x - drag_start_x
        dy = y - drag_start_y
        pan_x += dx
        pan_y += dy
        drag_start_x = x
        drag_start_y = y
        update_display_image()
    elif event == cv2.EVENT_LBUTTONUP:
        is_dragging = False

def main():
    parser = argparse.ArgumentParser(description="Compare Original and Synthetic Depth Maps")
    parser.add_argument("--parent_folder", type=str, required=True,
                        help="Parent folder containing 'depth_maps' and output folders")
    parser.add_argument("--file_id", type=str, required=True,
                        help="File identifier (e.g., 0000000931)")
    parser.add_argument("--synthetic_output_folder", type=str, default=None,
                        help="Name of synthetic output folder (auto-detected if not provided)")
    
    args = parser.parse_args()
    
    parent_folder = Path(args.parent_folder)
    if not parent_folder.exists():
        print(f"Error: Parent folder not found: {parent_folder}")
        return
    
    # 파일 경로 설정
    original_depth_path = parent_folder / "depth_maps" / f"{args.file_id}.png"
    
    # Synthetic output 폴더 자동 탐지
    if args.synthetic_output_folder:
        synthetic_base_dir = parent_folder.parent / args.synthetic_output_folder
    else:
        # synthetic_depth_output_* 패턴으로 폴더 찾기
        synthetic_folders = list(parent_folder.parent.glob("synthetic_depth_output_*"))
        if not synthetic_folders:
            print("Error: No synthetic depth output folders found")
            return
        synthetic_base_dir = synthetic_folders[0]
        print(f"Auto-detected synthetic folder: {synthetic_base_dir.name}")
    
    synthetic_depth_path = synthetic_base_dir / "new_depth_map" / f"{args.file_id}.png"
    
    # 파일 존재 확인
    if not original_depth_path.exists():
        print(f"Error: Original depth map not found: {original_depth_path}")
        return
    
    if not synthetic_depth_path.exists():
        print(f"Error: Synthetic depth map not found: {synthetic_depth_path}")
        return
    
    # 전역 변수 설정
    global original_depth_map, synthetic_depth_map, original_color_mapped, synthetic_color_mapped
    global current_display_mode, zoom_level, pan_x, pan_y
    
    # 깊이 맵 로드
    print(f"Loading original depth map: {original_depth_path}")
    original_depth_map = load_depth_map_png(original_depth_path)
    
    print(f"Loading synthetic depth map: {synthetic_depth_path}")
    synthetic_depth_map = load_depth_map_png(synthetic_depth_path)
    
    if original_depth_map is None and synthetic_depth_map is None:
        print("Error: Could not load any depth maps")
        return
    
    # 통계 정보 출력
    if original_depth_map is not None:
        orig_valid = original_depth_map > 0
        print(f"Original - Valid pixels: {np.sum(orig_valid):,}, "
              f"Depth range: [{np.min(original_depth_map[orig_valid]):.2f}, {np.max(original_depth_map[orig_valid]):.2f}] m")
    
    if synthetic_depth_map is not None:
        synth_valid = synthetic_depth_map > 0
        print(f"Synthetic - Valid pixels: {np.sum(synth_valid):,}, "
              f"Depth range: [{np.min(synthetic_depth_map[synth_valid]):.2f}, {np.max(synthetic_depth_map[synth_valid]):.2f}] m")
    
    # 컬러맵 생성 (magma 사용)
    original_color_mapped = create_colorized_depth_map(original_depth_map, cv2.COLORMAP_MAGMA)
    synthetic_color_mapped = create_colorized_depth_map(synthetic_depth_map, cv2.COLORMAP_MAGMA)
    
    # 초기 설정
    current_display_mode = 'side_by_side'
    zoom_level = 1.0
    pan_x = 0.0
    pan_y = 0.0
    
    # OpenCV 창 설정
    cv2.namedWindow("Depth Map Comparison", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Depth Map Comparison", WINDOW_WIDTH, WINDOW_HEIGHT)
    cv2.setMouseCallback("Depth Map Comparison", mouse_callback)
    
    update_display_image()
    
    print("\n=== Controls ===")
    print("1: Show original depth map only")
    print("2: Show synthetic depth map only") 
    print("3: Show side-by-side comparison")
    print("4: Show difference map")
    print("Mouse scroll: Zoom in/out")
    print("Mouse drag: Pan")
    print("S: Save current view")
    print("Q: Quit")
    
    # 메인 루프
    while True:
        key = cv2.waitKey(30) & 0xFF
        
        if key == ord('q') or key == 27:  # Q or ESC
            break
        elif key == ord('1'):
            current_display_mode = 'original'
            update_display_image()
        elif key == ord('2'):
            current_display_mode = 'synthetic'
            update_display_image()
        elif key == ord('3'):
            current_display_mode = 'side_by_side'
            update_display_image()
        elif key == ord('4'):
            current_display_mode = 'difference'
            update_display_image()
        elif key == ord('s'):
            # 현재 화면 저장
            screenshot = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
            cv2.imshow("Depth Map Comparison", screenshot)  # Get current window content
            output_path = parent_folder / f"comparison_{args.file_id}_{current_display_mode}.png"
            # 실제로는 현재 표시된 이미지를 저장해야 하지만, 간단히 컬러맵 저장
            if current_display_mode == 'side_by_side' and original_color_mapped is not None and synthetic_color_mapped is not None:
                h, w = original_color_mapped.shape[:2]
                save_img = np.zeros((h, w * 2, 3), dtype=np.uint8) + 255
                save_img[:, :w] = original_color_mapped
                save_img[:, w:] = synthetic_color_mapped
                cv2.line(save_img, (w, 0), (w, h-1), (0, 0, 0), 2)
                cv2.imwrite(str(output_path), save_img)
            elif current_display_mode == 'original' and original_color_mapped is not None:
                cv2.imwrite(str(output_path), original_color_mapped)
            elif current_display_mode == 'synthetic' and synthetic_color_mapped is not None:
                cv2.imwrite(str(output_path), synthetic_color_mapped)
            elif current_display_mode == 'difference':
                diff_img = create_difference_map(original_depth_map, synthetic_depth_map)
                if diff_img is not None:
                    cv2.imwrite(str(output_path), diff_img)
            print(f"Saved: {output_path}")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
