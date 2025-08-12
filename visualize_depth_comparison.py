import os
import json
import numpy as np
import cv2
import sys
import open3d as o3d
import pathlib
from ref.ref_calibration_data import DEFAULT_CALIB
from PIL import Image, ImageDraw # Added for Image and ImageDraw
from ref.ref_camera_lidar_projector import CalibrationDB, LidarCameraProjector # Added for CalibrationDB and LidarCameraProjector
from typing import Optional, List, Tuple # Added for Optional
from scipy.spatial import KDTree

# 전역 변수 초기화 (필요하다면)
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

def visualize_and_save_pcd_as_png(pcd_file_path: pathlib.Path, output_png_path: pathlib.Path):
    """
    Open3D를 사용하여 PCD 파일을 로드하고 시각화한 후, 해당 뷰를 PNG 이미지로 저장합니다.
    """
    print(f"PCD 파일 로드 및 시각화: {pcd_file_path}")
    try:
        pcd = o3d.io.read_point_cloud(str(pcd_file_path))
        if not pcd.has_points():
            print(f"경고: {pcd_file_path}에 포인트가 없습니다.")
            return

        vis = o3d.visualization.Visualizer()
        vis.create_window(width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False) # 창을 보이지 않게 생성
        vis.add_geometry(pcd)

        # 렌더링 옵션 설정 (예: 포인트 크기)
        render_option = vis.get_render_option()
        render_option.point_size = 1.0
        render_option.background_color = np.asarray([1, 1, 1]) # 흰색 배경

        vis.poll_events()
        vis.update_renderer()

        # 이미지 저장
        vis.capture_screen_image(str(output_png_path))
        print(f"PCD 시각화 이미지를 {output_png_path}에 저장했습니다.")

        vis.destroy_window()
    except Exception as e:
        print(f"PCD 시각화 및 PNG 저장 중 오류 발생 ({pcd_file_path}): {e}", file=sys.stderr)

def visualize_pcd_comparison(
    original_pcd_path: pathlib.Path,
    augmented_pcd_path: pathlib.Path,
    output_png_path: pathlib.Path
):
    """
    원본 PCD와 증강된 PCD를 로드하고 다른 색상으로 시각화한 후, 해당 뷰를 PNG 이미지로 저장합니다.
    """
    print(f"PCD 비교 시각화: 원본 {original_pcd_path}, 증강 {augmented_pcd_path}")
    try:
        original_pcd = o3d.io.read_point_cloud(str(original_pcd_path))
        augmented_pcd = o3d.io.read_point_cloud(str(augmented_pcd_path))

        if not original_pcd.has_points():
            print(f"경고: 원본 PCD ({original_pcd_path})에 포인트가 없습니다.")
            return
        if not augmented_pcd.has_points():
            print(f"경고: 증강된 PCD ({augmented_pcd_path})에 포인트가 없습니다.")
            return

        # 원본 PCD에 회색 할당
        original_pcd.paint_uniform_color([0.5, 0.5, 0.5]) # Gray

        # 증강된 PCD에 빨간색 할당
        augmented_pcd.paint_uniform_color([1.0, 0.0, 0.0]) # Red

        vis = o3d.visualization.Visualizer()
        vis.create_window(width=WINDOW_WIDTH, height=WINDOW_HEIGHT, visible=False)
        vis.add_geometry(original_pcd)
        vis.add_geometry(augmented_pcd)

        render_option = vis.get_render_option()
        render_option.point_size = 2.0 # 포인트 크기 증가
        render_option.background_color = np.asarray([1, 1, 1]) # 흰색 배경

        # 카메라 뷰 설정 (두 PCD가 잘 보이도록 동적으로 조정)
        # 두 PCD를 합쳐서 경계 상자를 계산
        combined_pcd = original_pcd + augmented_pcd
        if combined_pcd.has_points():
            bbox = combined_pcd.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_max_bound() - bbox.get_min_bound()
            max_extent = np.max(extent)

            view_control = vis.get_view_control()
            # 카메라 위치를 경계 상자 중앙에서 적절한 거리만큼 떨어뜨려 설정
            # Z축을 위로, Y축을 앞으로 보는 시점 (일반적인 LiDAR 뷰)
            camera_distance = max_extent * 1.5 # 경계 상자 크기에 비례하여 거리 설정
            view_control.set_lookat(center)
            view_control.set_front([0, -1, 0]) # Y축 방향으로 바라봄
            view_control.set_up([0, 0, 1])    # Z축이 위를 향하도록
            view_control.set_zoom(0.8 / camera_distance) # 거리에 따라 줌 레벨 조정

        vis.poll_events()
        vis.update_renderer()

        vis.capture_screen_image(str(output_png_path))
        print(f"PCD 비교 시각화 이미지를 {output_png_path}에 저장했습니다.")

        vis.destroy_window()

    except Exception as e:
        print(f"PCD 비교 시각화 및 PNG 저장 중 오류 발생: {e}", file=sys.stderr)

def _get_all_projected_data(
    projector: LidarCameraProjector, 
    pcd: o3d.geometry.PointCloud, 
    image_size: tuple
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Projects all valid points and returns their screen, camera, and world coordinates."""
    world_coords = np.asarray(pcd.points)
    
    # Project points and get their coordinates
    # Returns list of (u, v, Xc, Yc, Zc, original_index)
    valid_projections = projector.get_valid_projections_with_camera_coords("a6", world_coords, image_size)
    
    if not valid_projections:
        return np.empty((0, 2)), np.empty((0, 3)), np.empty((0, 3))

    # Unzip the data
    screen_coords_list = []
    camera_coords_list = []
    world_coords_list = []
    
    for u, v, xc, yc, zc, original_index in valid_projections:
        screen_coords_list.append([u, v])
        camera_coords_list.append([xc, yc, zc])
        world_coords_list.append(world_coords[original_index])

    return np.array(screen_coords_list), np.array(camera_coords_list), np.array(world_coords_list)


def _draw_projected_points(
    draw: ImageDraw.Draw, 
    projector: LidarCameraProjector, 
    pcd: o3d.geometry.PointCloud, 
    image_size: tuple, 
    color_override: Optional[str] = None,
    point_radius: int = 2,
    max_depth_for_purple: Optional[float] = None
):
    """Helper function to project and draw PCD points onto an image."""
    points_xyz = np.asarray(pcd.points)
    projected_data = projector.get_valid_projections_with_camera_coords("a6", points_xyz, image_size)
    
    for u, v, Xc, _, _, _ in projected_data:
        # max_depth 필터링 (color_override와 무관하게 적용)
        if max_depth_for_purple is not None and Xc > max_depth_for_purple:
            continue # Skip drawing if depth exceeds the limit
            
        if color_override == "purple":
            color = (128, 0, 128) # RGB for purple
        elif color_override == "red":
            color = (255, 0, 0) # RGB for red
        elif color_override == "black":
            color = (0, 0, 0) # RGB for black
        else:
            color = projector._get_color_from_distance(Xc)
        draw.ellipse((u - point_radius, v - point_radius, u + point_radius, v + point_radius), fill=color, outline=color)

def visualize_original_and_line_projection(
    original_pcd: o3d.geometry.PointCloud,
    line_points: np.ndarray,
    image: Image.Image,
    calib_db: CalibrationDB,
    output_path: pathlib.Path
):
    """
    원본 PCD와 추출된 라인 포인트를 이미지에 투영하여 비교 시각화하고 PNG로 저장합니다.
    """
    print(f"이미지 투영 비교 시각화 (원본 vs 라인): {output_path}...")
    debug_image = image.copy()
    draw = ImageDraw.Draw(debug_image)

    projector = LidarCameraProjector(calib_db, max_range_m=10.0)

    # 원본 PCD 투영 (거리에 따른 색상)
    _draw_projected_points(draw, projector, original_pcd, image.size)

    # 라인 포인트 투영 (빨간색으로 오버라이드)
    if line_points is not None and line_points.shape[0] > 0:
        line_pcd = o3d.geometry.PointCloud()
        line_pcd.points = o3d.utility.Vector3dVector(line_points)
        _draw_projected_points(draw, projector, line_pcd, image.size, color_override="red")

    debug_image.save(output_path)
    print(f"이미지 투영 비교 시각화 이미지를 {output_path}에 저장했습니다.")

def show_interactive_projected_pcd_viewer(
    original_pcd: o3d.geometry.PointCloud,
    line_points: np.ndarray,
    base_image: Image.Image,
    calib_db: CalibrationDB,
    depth_filtered_pcd: Optional[o3d.geometry.PointCloud] = None,
    depth_threshold: float = 5.0  # 새로운 파라미터 추가
):
    """
    원본 PCD와 추출된 라인 포인트를 이미지에 투영하여 인터랙티브하게 시각화합니다.
    마우스 휠로 줌, 드래그로 패닝, 호버 시 좌표 정보를 표시합니다.
    
    Args:
        original_pcd: 원본 포인트 클라우드
        line_points: 추출된 라인 포인트들
        base_image: 베이스 이미지
        calib_db: 캘리브레이션 데이터베이스
        depth_filtered_pcd: 깊이 필터링된 포인트 클라우드 (선택사항)
        depth_threshold: 깊이 필터링 임계값 (미터)
    """
    print(f"인터랙티브 투영 PCD 뷰어 시작. 깊이 임계값: {depth_threshold}m")
    print("(Q: 종료, P: 모드 전환, 마우스 휠: 줌, 드래그: 패닝)")

    # --- 상태 변수 ---
    local_zoom_level = 1.0
    local_pan_x, local_pan_y = 0.0, 0.0
    local_drag_start_x, local_drag_start_y = 0, 0
    local_is_dragging = False
    local_display_image_cv = None
    local_display_mode = "ALL" # "ALL" 또는 "DEPTH_FILTERED"
    
    # 미리 렌더링된 이미지 저장 변수
    local_all_pcds_image_cv = None
    local_depth_filtered_pcd_image_cv = None
    
    # --- 데이터 준비 ---
    projector = LidarCameraProjector(calib_db, max_range_m=10.0)
    
    # 모든 점의 투영 좌표, 카메라 좌표, 월드 좌표를 미리 계산
    all_screen_coords, all_camera_coords, all_world_coords = _get_all_projected_data(
        projector, original_pcd, base_image.size
    )
    
    # 화면 좌표를 기반으로 KDTree 생성 (빠른 검색용)
    kdtree = None
    if all_screen_coords.shape[0] > 0:
        kdtree = KDTree(all_screen_coords)

    # --- 초기 이미지 렌더링 (각 모드별로 한 번만 수행) ---
    # "ALL" 모드 이미지 렌더링
    all_pcds_pil_image = base_image.copy()
    draw_all = ImageDraw.Draw(all_pcds_pil_image)
    _draw_projected_points(draw_all, projector, original_pcd, base_image.size, point_radius=2)
    if line_points is not None and line_points.shape[0] > 0:
        line_pcd = o3d.geometry.PointCloud()
        line_pcd.points = o3d.utility.Vector3dVector(line_points)
        _draw_projected_points(draw_all, projector, line_pcd, base_image.size, color_override="red", point_radius=3)
    
    # depth_threshold 이내 점들을 보라색으로 오버레이
    if depth_filtered_pcd is not None and depth_filtered_pcd.has_points():
        _draw_projected_points(draw_all, projector, original_pcd, base_image.size, color_override="purple", point_radius=3, max_depth_for_purple=depth_threshold)
    local_all_pcds_image_cv = cv2.cvtColor(np.array(all_pcds_pil_image.convert("RGB")), cv2.COLOR_RGB2BGR)

    # "DEPTH_FILTERED" 모드 이미지 렌더링
    depth_filtered_pil_image = base_image.copy()
    draw_depth_filtered = ImageDraw.Draw(depth_filtered_pil_image)
    # 원본 PCD에서 depth_threshold 이내 점들만 보라색으로 그리기
    _draw_projected_points(draw_depth_filtered, projector, original_pcd, base_image.size, color_override="purple", point_radius=3, max_depth_for_purple=depth_threshold)
    local_depth_filtered_pcd_image_cv = cv2.cvtColor(np.array(depth_filtered_pil_image.convert("RGB")), cv2.COLOR_RGB2BGR)

    def update_local_display_image():
        """줌/패닝 상태에 따라 화면에 표시될 이미지를 업데이트합니다."""
        nonlocal local_display_image_cv, local_pan_x, local_pan_y, local_display_mode, local_all_pcds_image_cv, local_depth_filtered_pcd_image_cv
        
        # 현재 모드에 따라 사용할 원본 이미지 선택
        if local_display_mode == "ALL":
            current_original_image_cv = local_all_pcds_image_cv
        elif local_display_mode == "DEPTH_FILTERED":
            current_original_image_cv = local_depth_filtered_pcd_image_cv
        else:
            current_original_image_cv = None # Fallback, should not happen

        if current_original_image_cv is None: return

        h_orig, w_orig, _ = current_original_image_cv.shape
        new_w, new_h = int(w_orig * local_zoom_level), int(h_orig * local_zoom_level)
        
        zoomed_image = cv2.resize(current_original_image_cv, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # 패닝 범위 제한
        if new_w < WINDOW_WIDTH: local_pan_x = (WINDOW_WIDTH - new_w) / 2
        else: local_pan_x = max(min(local_pan_x, 0.0), -(new_w - WINDOW_WIDTH))
        if new_h < WINDOW_HEIGHT: local_pan_y = (WINDOW_HEIGHT - new_h) / 2
        else: local_pan_y = max(min(local_pan_y, 0.0), -(new_h - WINDOW_HEIGHT))

        # 캔버스에 줌/패닝된 이미지 그리기
        canvas = np.full((WINDOW_HEIGHT, WINDOW_WIDTH, 3), 255, dtype=np.uint8)
        src_x1, src_y1 = max(0, -int(local_pan_x)), max(0, -int(local_pan_y))
        src_x2, src_y2 = min(new_w, src_x1 + WINDOW_WIDTH), min(new_h, src_y1 + WINDOW_HEIGHT)
        dst_x1, dst_y1 = max(0, int(local_pan_x)), max(0, int(local_pan_y))
        dst_x2, dst_y2 = min(WINDOW_WIDTH, dst_x1 + (src_x2 - src_x1)), min(WINDOW_HEIGHT, dst_y1 + (src_y2 - src_y1))
        
        if (dst_y2 > dst_y1) and (dst_x2 > dst_x1):
            canvas[dst_y1:dst_y2, dst_x1:dst_x2] = zoomed_image[src_y1:src_y2, src_x1:src_x2]

        local_display_image_cv = canvas
        cv2.imshow("Interactive Projected PCD Viewer", local_display_image_cv)

    def mouse_callback_local(event, x, y, flags, param):
        nonlocal local_zoom_level, local_pan_x, local_pan_y, local_drag_start_x, local_drag_start_y, local_is_dragging

        if event == cv2.EVENT_MOUSEWHEEL:
            zoom_factor = 1.1 if flags > 0 else 1 / 1.1
            new_zoom_level = max(0.1, min(10.0, local_zoom_level * zoom_factor))
            if new_zoom_level != local_zoom_level:
                orig_x_at_mouse = (x - local_pan_x) / local_zoom_level
                orig_y_at_mouse = (y - local_pan_y) / local_zoom_level
                local_zoom_level = new_zoom_level
                local_pan_x = x - (orig_x_at_mouse * local_zoom_level)
                local_pan_y = y - (orig_y_at_mouse * local_zoom_level)
                update_local_display_image()

        elif event == cv2.EVENT_LBUTTONDOWN:
            local_is_dragging = True
            local_drag_start_x, local_drag_start_y = x, y

        elif event == cv2.EVENT_MOUSEMOVE:
            if local_is_dragging:
                dx, dy = x - local_drag_start_x, y - local_drag_start_y
                local_pan_x += dx
                local_pan_y += dy
                local_drag_start_x, local_drag_start_y = x, y
                update_local_display_image()
            elif kdtree:
                # 드래그 중이 아닐 때만 좌표 정보 표시
                img_with_text = local_display_image_cv.copy()
                
                # 화면 좌표(x,y)를 원본 이미지 좌표로 변환
                orig_x = (x - local_pan_x) / local_zoom_level
                orig_y = (y - local_pan_y) / local_zoom_level
                
                dist, idx = kdtree.query([orig_x, orig_y], k=1)
                
                # 줌 레벨을 고려하여, 화면상에서 5픽셀 반경 내에 점이 있으면 정보 표시
                if dist < 5 / local_zoom_level:
                    cam_coords = all_camera_coords[idx]
                    world_coords = all_world_coords[idx]
                    
                    depth_text = f"Depth: {cam_coords[0]:.2f} m"
                    coord_text = f"XYZ: {world_coords[0]:.2f}, {world_coords[1]:.2f}, {world_coords[2]:.2f}"
                    
                    # 텍스트 스타일
                    font, scale, color, thickness = cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
                    
                    # 텍스트 위치 계산 및 배경 사각형 그리기
                    (w1, h1), _ = cv2.getTextSize(depth_text, font, scale, thickness)
                    (w2, h2), _ = cv2.getTextSize(coord_text, font, scale, thickness)
                    
                    # 텍스트가 창 밖으로 나가지 않도록 위치 조정
                    text_x = x + 20
                    text_y = y + 20
                    if text_x + max(w1, w2) > WINDOW_WIDTH: text_x = x - max(w1, w2) - 20
                    if text_y + h1 + h2 + 30 > WINDOW_HEIGHT: text_y = y - h1 - h2 - 30

                    cv2.rectangle(img_with_text, (text_x - 5, text_y - h1 - 5), (text_x + w1 + 5, text_y + 5), (255, 255, 255), -1)
                    cv2.putText(img_with_text, depth_text, (text_x, text_y), font, scale, color, thickness)
                    
                    cv2.rectangle(img_with_text, (text_x - 5, text_y + 20 - h2 - 5), (text_x + w2 + 5, text_y + 20 + 5), (255, 255, 255), -1)
                    cv2.putText(img_with_text, coord_text, (text_x, text_y + 20), font, scale, color, thickness)

                cv2.imshow("Interactive Projected PCD Viewer", img_with_text)

        elif event == cv2.EVENT_LBUTTONUP:
            local_is_dragging = False
            update_local_display_image() # 드래그 종료 후 텍스트 없는 깨끗한 이미지로 갱신

    # --- 창 설정 및 메인 루프 ---
    cv2.namedWindow("Interactive Projected PCD Viewer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Interactive Projected PCD Viewer", WINDOW_WIDTH, WINDOW_HEIGHT)
    cv2.setMouseCallback("Interactive Projected PCD Viewer", mouse_callback_local)

    update_local_display_image()

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            if local_display_mode == "ALL":
                local_display_mode = "DEPTH_FILTERED"
                print("Display Mode: Depth Filtered PCD Only")
            else:
                local_display_mode = "ALL"
                print("Display Mode: All PCDs")
            update_local_display_image()

    cv2.destroyAllWindows()
    print("인터랙티브 뷰어 종료.")