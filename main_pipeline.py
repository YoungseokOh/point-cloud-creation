import numpy as np
import open3d as o3d
from pathlib import Path
from typing import Optional
from visualize_depth_comparison import visualize_pcd_comparison, visualize_original_and_line_projection, show_interactive_projected_pcd_viewer
from ref.ref_camera_lidar_projector import CalibrationDB, load_image, LidarCameraProjector
from ref.ref_calibration_data import DEFAULT_CALIB, DEFAULT_LIDAR_TO_WORLD_v2

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

def filter_pcd_by_depth(
    pcd: o3d.geometry.PointCloud, 
    calib_db: CalibrationDB, 
    max_depth: float
) -> Optional[o3d.geometry.PointCloud]:
    """Filters a point cloud based on a maximum depth value in the camera's view."""
    print(f"Filtering PCD to include only points within {max_depth} meters.")
    if not pcd.has_points():
        print("Error: Input point cloud is empty.")
        return None

    # The projector needs an image size, but we only need the projection math.
    # We can provide a dummy size.
    dummy_image_size = (1920, 1536) 
    projector = LidarCameraProjector(calib_db)
    
    points_xyz = np.asarray(pcd.points)
    
    # Get projected data which includes camera coordinates (Xc, Yc, Zc)
    # The list contains tuples of (u, v, Xc, Yc, Zc, original_index)
    projected_data = projector.get_valid_projections_with_camera_coords("a6", points_xyz, dummy_image_size)
    
    if not projected_data:
        print("Warning: No valid projections found for the given PCD.")
        return None

    # Filter indices based on depth (Xc)
    filtered_indices = [
        original_index for u, v, Xc, Yc, Zc, original_index in projected_data if Xc <= max_depth and Xc > 0
    ]

    if not filtered_indices:
        print(f"Warning: No points found within the depth threshold of {max_depth} meters.")
        return None

    # Create a new point cloud with the filtered points
    filtered_pcd = pcd.select_by_index(filtered_indices)
    print(f"Found {len(filtered_pcd.points)} points within {max_depth}m depth.")
    return filtered_pcd

# --- Main Execution Block ---
from typing import Optional
from visualize_depth_comparison import visualize_pcd_comparison, visualize_original_and_line_projection, show_interactive_projected_pcd_viewer
from ref.ref_camera_lidar_projector import CalibrationDB, load_image, LidarCameraProjector
from ref.ref_calibration_data import DEFAULT_CALIB, DEFAULT_LIDAR_TO_WORLD_v2

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
    DEPTH_THRESHOLD_METERS = 2.55 # Depth 필터링을 위한 거리 임계값 (미터)

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
        print(f"원본 PCD 포인트 수: {len(original_pcd.points)}")
        
        # Depth 기준으로 PCD 필터링
        depth_filtered_pcd = filter_pcd_by_depth(original_pcd, calib_db, DEPTH_THRESHOLD_METERS)
        
        if depth_filtered_pcd:
            print(f"필터링된 PCD 포인트 수: {len(depth_filtered_pcd.points)}")
            reduction_ratio = len(depth_filtered_pcd.points) / len(original_pcd.points) * 100
            print(f"필터링 후 포인트 비율: {reduction_ratio:.1f}%")
        
        # X 값 기준으로 라인 포인트 추출
        line_points = find_lidar_line_by_x(original_pcd, TARGET_X_VALUE, TOLERANCE)
        
        if line_points is not None:
            print(f"라인 포인트 수: {len(line_points)}")
            
            # 인터랙티브 뷰어 실행 - depth_threshold 파라미터 전달
            show_interactive_projected_pcd_viewer(
                original_pcd, 
                line_points, 
                image, 
                calib_db, 
                depth_filtered_pcd,
                depth_threshold=DEPTH_THRESHOLD_METERS  # 파라미터 전달
            )
        else:
            print("라인 포인트를 찾을 수 없습니다.")
    else:
        print("PCD 파일을 로드할 수 없습니다.")

    print("\n스크립트 실행 완료.")
