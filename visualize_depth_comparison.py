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
        render_option.point_size = 1.0
        render_option.background_color = np.asarray([1, 1, 1]) # 흰색 배경

        # 카메라 뷰 설정 (두 PCD가 잘 보이도록 조정)
        view_control = vis.get_view_control()
        view_control.set_front([0, -1, 0]) # Y축을 위로
        view_control.set_up([0, 0, 1])    # Z축을 앞으로
        view_control.set_zoom(0.7)        # 적절한 줌 레벨

        vis.poll_events()
        vis.update_renderer()

        vis.capture_screen_image(str(output_png_path))
        print(f"PCD 비교 시각화 이미지를 {output_png_path}에 저장했습니다.")

        vis.destroy_window()

    except Exception as e:
        print(f"PCD 비교 시각화 및 PNG 저장 중 오류 발생: {e}", file=sys.stderr)

def visualize_projected_comparison(
    original_pcd: o3d.geometry.PointCloud,
    augmented_points: np.ndarray,
    image: Image.Image,
    calib_db: CalibrationDB,
    output_path: pathlib.Path
):
    """
    원본 PCD와 증강된 포인트를 이미지에 투영하여 비교 시각화하고 PNG로 저장합니다.
    원본 포인트는 파란색, 증강된 포인트는 초록색으로 표시됩니다.
    """
    print(f"이미지 투영 비교 시각화: {output_path}...")
    debug_image = image.copy()
    draw = ImageDraw.Draw(debug_image)

    projector = LidarCameraProjector(calib_db)

    # 원본 PCD 투영
    original_cloud_xyz = np.asarray(original_pcd.points)
    original_projected_data = projector.get_valid_projections_with_camera_coords("a6", original_cloud_xyz, image.size)
    for u, v, _, _, _, _ in original_projected_data:
        draw.point((u, v), fill="blue") # 원본 포인트는 파란색

    # 증강된 포인트 투영
    if augmented_points is not None and augmented_points.shape[0] > 0:
        augmented_projected_data = projector.get_valid_projections_with_camera_coords("a6", augmented_points, image.size)
        for u, v, _, _, _, _ in augmented_projected_data:
            draw.point((u, v), fill="green") # 증강된 포인트는 초록색

    debug_image.save(output_path)
    print(f"이미지 투영 비교 시각화 이미지를 {output_path}에 저장했습니다.")