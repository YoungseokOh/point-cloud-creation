import os
import json
import numpy as np
import cv2
import sys
import open3d as o3d
import pathlib
from ref.calibration_data import DEFAULT_CALIB
from PIL import Image, ImageDraw # Added for Image and ImageDraw
from ref.camera_lidar_projector import CalibrationDB, LidarCameraProjector # Added for CalibrationDB and LidarCameraProjector
from typing import Optional # Added for Optional

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

def _draw_projected_points(draw: ImageDraw.Draw, projector: LidarCameraProjector, pcd: o3d.geometry.PointCloud, image_size: tuple, color_override: Optional[str] = None):
    """Helper function to project and draw PCD points onto an image."""
    points_xyz = np.asarray(pcd.points)         
                
    projected_data = projector.get_valid_projections_with_camera_coords("a6", points_xyz, image_size)
    
    for u, v, Xc, _, _, _ in projected_data:
        color = color_override if color_override else projector._get_color_from_distance(Xc)
        # draw.point((u, v), fill=color)
        draw.ellipse((u - 2, v - 2, u + 2, v + 2), fill=color, outline=color)

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
    calib_db: CalibrationDB
):
    """
    원본 PCD와 추출된 라인 포인트를 이미지에 투영하여 인터랙티브하게 시각화합니다.
    마우스 휠로 줌, 드래그로 패닝이 가능합니다.
    """
    print("인터랙티브 투영 PCD 뷰어 시작. (Q: 종료, 마우스 휠: 줌, 드래그: 패닝)")

    # 로컬 상태 변수
    local_zoom_level = 1.0
    local_pan_x = 0.0
    local_pan_y = 0.0
    local_drag_start_x = 0
    local_drag_start_y = 0
    local_is_dragging = False
    local_display_image_cv = None # OpenCV 형식의 현재 표시 이미지
    local_original_projected_image_cv = None # 원본 투영 이미지 (줌/패닝 전)

    # 초기 투영 이미지 생성
    projector = LidarCameraProjector(calib_db, max_range_m=10.0)
    projected_pil_image = base_image.copy()
    draw = ImageDraw.Draw(projected_pil_image)

    # 원본 PCD 투영 (거리에 따른 색상)
    _draw_projected_points(draw, projector, original_pcd, base_image.size)

    # 라인 포인트 투영 (빨간색으로 오버라이드)
    if line_points is not None and line_points.shape[0] > 0:
        line_pcd = o3d.geometry.PointCloud()
        line_pcd.points = o3d.utility.Vector3dVector(line_points)
        _draw_projected_points(draw, projector, line_pcd, base_image.size, color_override="red")

    local_original_projected_image_cv = cv2.cvtColor(np.array(projected_pil_image.convert("RGB")), cv2.COLOR_RGB2BGR)

    def update_local_display_image():
        nonlocal local_display_image_cv, local_zoom_level, local_pan_x, local_pan_y, local_original_projected_image_cv

        if local_original_projected_image_cv is None:
            return

        h_orig, w_orig, _ = local_original_projected_image_cv.shape
        new_w, new_h = int(w_orig * local_zoom_level), int(h_orig * local_zoom_level)
        
        zoomed_image = cv2.resize(local_original_projected_image_cv, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        if new_w < WINDOW_WIDTH:
            local_pan_x = (WINDOW_WIDTH - new_w) / 2
        else:
            local_pan_x = max(min(local_pan_x, 0.0), -(new_w - WINDOW_WIDTH))

        if new_h < WINDOW_HEIGHT:
            local_pan_y = (WINDOW_HEIGHT - new_h) / 2
        else:
            local_pan_y = max(min(local_pan_y, 0.0), -(new_h - WINDOW_HEIGHT))

        canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8) + 255

        src_x1 = max(0, -int(local_pan_x))
        src_y1 = max(0, -int(local_pan_y))
        src_x2 = min(new_w, src_x1 + WINDOW_WIDTH)
        src_y2 = min(new_h, src_y1 + WINDOW_HEIGHT)

        dst_x1 = max(0, int(local_pan_x))
        dst_y1 = max(0, int(local_pan_y))
        dst_x2 = min(WINDOW_WIDTH, dst_x1 + (src_x2 - src_x1))
        dst_y2 = min(WINDOW_HEIGHT, dst_y1 + (src_y2 - src_y1))

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
            local_drag_start_x = x
            local_drag_start_y = y
        elif event == cv2.EVENT_MOUSEMOVE and local_is_dragging:
            dx = x - local_drag_start_x
            dy = y - local_drag_start_y
            local_pan_x += dx
            local_pan_y += dy
            local_drag_start_x = x
            local_drag_start_y = y
            update_local_display_image()
        elif event == cv2.EVENT_LBUTTONUP:
            local_is_dragging = False

    cv2.namedWindow("Interactive Projected PCD Viewer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Interactive Projected PCD Viewer", WINDOW_WIDTH, WINDOW_HEIGHT)
    cv2.setMouseCallback("Interactive Projected PCD Viewer", mouse_callback_local)

    update_local_display_image()

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): # 'q'를 누르면 종료
            break

    cv2.destroyWindow("Interactive Projected PCD Viewer")
    print("인터랙티브 뷰어 종료.")