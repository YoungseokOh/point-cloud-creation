import numpy as np
import open3d as o3d
from pathlib import Path
from typing import Optional

# --- Core Functions ---
def load_lidar_pcd(pcd_path: Path) -> Optional[o3d.geometry.PointCloud]:
    """Loads a PCD file and handles potential errors."""
    print(f"Attempting to load PCD file: {pcd_path}")
    if not pcd_path.exists():
        print(f"Error: PCD file not found at {pcd_path}")
        return None
    try:
        pcd = o3d.io.read_point_cloud(str(pcd_path))
        if not pcd.has_points():
            print(f"Error: PCD file is empty or could not be read: {pcd_path}")
            return None
        print(f"Successfully loaded PCD: {pcd_path} with {len(pcd.points)} points.")
        return pcd
    except Exception as e:
        print(f"Error loading PCD file {pcd_path}: {e}")
        return None

# --- LiDAR Line Finding Function ---
def find_lidar_line_by_x(pcd: o3d.geometry.PointCloud, x_value: float, tolerance: float = 0.01) -> Optional[np.ndarray]:
    """Finds points in the PCD that fall within a narrow X-coordinate range, representing a line.

    Args:
        pcd: The Open3D PointCloud object.
        x_value: The target X-coordinate value for the line.
        tolerance: The tolerance around x_value to consider points part of the line.

    Returns:
        A NumPy array of (N, 3) representing the XYZ coordinates of the points on the line,
        or None if no points are found.
    """
    print(f"Finding LiDAR line around X={x_value} with tolerance {tolerance}...")
    if not pcd.has_points():
        print("Error: Point cloud is empty.")
        return None

    points_xyz = np.asarray(pcd.points)

    # X 값 범위 내에 있는 포인트 필터링
    min_x = x_value - tolerance
    max_x = x_value + tolerance

    line_indices = np.where((points_xyz[:, 0] >= min_x) & (points_xyz[:, 0] <= max_x))[0]

    if len(line_indices) == 0:
        print(f"Warning: No points found within X range [{min_x}, {max_x}].")
        return None

    line_points = points_xyz[line_indices]
    print(f"Found {len(line_points)} points for the LiDAR line.")
    return line_points

# --- Main Execution Block ---
from typing import Optional
from visualize_depth_comparison import visualize_pcd_comparison, visualize_original_and_line_projection, show_interactive_projected_pcd_viewer
from ref.camera_lidar_projector import CalibrationDB, DEFAULT_CALIB, DEFAULT_LIDAR_TO_WORLD_v2, load_image # load_image, CalibrationDB, DEFAULT_CALIB, DEFAULT_LIDAR_TO_WORLD 임포트 추가

if __name__ == "__main__":
    PROJECT_ROOT = Path("C:\\Users\\seok436\\Documents\\VSCode\\Projects\\point-cloud-creation\\point-cloud-creation")
    INPUT_DATA_DIR = PROJECT_ROOT / "ncdb-cls-sample" / "synced_data"
    INPUT_PCD_DIR = INPUT_DATA_DIR / "pcd"
    INPUT_IMG_DIR = INPUT_DATA_DIR / "image_a6" # 이미지 디렉토리 추가
    OUTPUT_DIR = PROJECT_ROOT / "output" # 새로운 출력 폴더 정의
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True) # 출력 폴더 생성

    TARGET_PCD_FILENAME = "0000001968.pcd"
    TARGET_IMAGE_FILENAME = "0000001968.jpg" # 이미지 파일명 추가
    TARGET_X_VALUE = -3.0 # 예시 X 값, 실제 PCD 데이터에 따라 조정 필요
    TOLERANCE = 0.05 # 예시 허용 오차

    input_pcd_path = INPUT_PCD_DIR / TARGET_PCD_FILENAME
    original_pcd = load_lidar_pcd(input_pcd_path)

    input_img_path = INPUT_IMG_DIR / TARGET_IMAGE_FILENAME # 이미지 경로
    try:
        image = load_image(input_img_path)
        print(f"Successfully loaded image: {input_img_path}")
    except FileNotFoundError:
        print(f"Error: Could not load image from {input_img_path}.")
        exit()

    calib_db = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v2) # 캘리브레이션 데이터베이스 로드

    if original_pcd:
        lidar_line_points = find_lidar_line_by_x(original_pcd, TARGET_X_VALUE, TOLERANCE)

        if lidar_line_points is not None:
            line_pcd = o3d.geometry.PointCloud()
            line_pcd.points = o3d.utility.Vector3dVector(lidar_line_points)
            output_line_pcd_path = OUTPUT_DIR / "output_line_pcd.pcd" # 출력 폴더 사용
            o3d.io.write_point_cloud(str(output_line_pcd_path), line_pcd)
            print(f"LiDAR 라인 포인트가 {output_line_pcd_path}에 저장되었습니다.")

            # 원본 PCD와 추출된 라인 PCD를 3D로 비교 시각화
            comparison_3d_png_path = OUTPUT_DIR / "lidar_line_comparison_3d.png"
            visualize_pcd_comparison(input_pcd_path, output_line_pcd_path, comparison_3d_png_path)
            print(f"LiDAR 라인 3D 비교 시각화 이미지가 {comparison_3d_png_path}에 저장되었습니다.")

            # 원본과 추출된 라인 포인트의 이미지 투영 비교 시각화
            projected_comparison_png_path = OUTPUT_DIR / "lidar_line_projected_comparison.png"
            visualize_original_and_line_projection(original_pcd, lidar_line_points, image, calib_db, projected_comparison_png_path)
            print(f"LiDAR 라인 이미지 투영 비교 시각화 이미지가 {projected_comparison_png_path}에 저장되었습니다.")

            # 인터랙티브 뷰어 호출
            show_interactive_projected_pcd_viewer(original_pcd, lidar_line_points, image, calib_db)

        else:
            print("LiDAR 라인을 찾지 못했습니다.")
    else:
        print("PCD 파일을 로드할 수 없습니다.")

    print("\n스크립트 실행 완료.")
