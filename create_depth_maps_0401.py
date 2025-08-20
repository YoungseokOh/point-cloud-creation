import sys
import os
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
from tqdm import tqdm

import numpy as np
import cv2

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Warning: open3d not found. Falling back to basic ASCII PCD parser.", file=sys.stderr)

# --- ref_create_depth_maps.py에서 가져온 클래스 및 함수 ---

class CameraModelBase:
    """Base class for camera projection models."""
    def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
        raise NotImplementedError

class VADASFisheyeCameraModel(CameraModelBase):
    """VADAS Polynomial Fisheye Camera Model, assuming +X is forward."""
    def __init__(self, intrinsic: List[float], image_size: Optional[Tuple[int, int]] = None):
        if len(intrinsic) < 11:
            raise ValueError("VADAS intrinsic must have at least 11 parameters.")
        self.k = intrinsic[0:7]
        self.s = intrinsic[7]
        self.div = intrinsic[8]
        self.ux = intrinsic[9]
        self.uy = intrinsic[10]
        self.image_size = image_size

    def _poly_eval(self, coeffs: List[float], x: float) -> float:
        res = 0.0
        for c in reversed(coeffs):
            res = res * x + c
        return res

    def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
        nx = -Yc
        ny = -Zc
        dist = math.hypot(nx, ny)
        if dist < sys.float_info.epsilon:
            dist = sys.float_info.epsilon
        cosPhi = nx / dist
        sinPhi = ny / dist
        theta = math.atan2(dist, Xc)

        if Xc < 0:
            return 0, 0, False

        xd = theta * self.s
        if abs(self.div) < 1e-9:
            return 0, 0, False
        
        rd = self._poly_eval(self.k, xd) / self.div
        if math.isinf(rd) or math.isnan(rd):
            return 0, 0, False

        img_w_half = (self.image_size[0] / 2) if self.image_size else 0
        img_h_half = (self.image_size[1] / 2) if self.image_size else 0

        u = rd * cosPhi + self.ux + img_w_half
        v = rd * sinPhi + self.uy + img_h_half
        
        return int(round(u)), int(round(v)), True

class SensorInfo:
    """Holds camera sensor information."""
    def __init__(self, name: str, model: CameraModelBase, intrinsic: List[float], extrinsic: np.ndarray, image_size: Optional[Tuple[int, int]] = None):
        self.name = name
        self.model = model
        self.intrinsic = intrinsic
        self.extrinsic = extrinsic
        self.image_size = image_size

class CalibrationDB:
    """Manages camera calibration data."""
    def __init__(self, calib_dict: Dict[str, Any], lidar_to_world: Optional[np.ndarray] = None):
        self.sensors: Dict[str, SensorInfo] = {}
        self.lidar_to_world = lidar_to_world if lidar_to_world is not None else np.eye(4)

        for cam_name, calib_data in calib_dict.items():
            model_type = calib_data["model"]
            intrinsic = calib_data["intrinsic"]
            extrinsic_raw = calib_data["extrinsic"]
            image_size = tuple(calib_data["image_size"]) if "image_size" in calib_data and calib_data["image_size"] else None

            extrinsic_matrix = self._rodrigues_to_matrix(extrinsic_raw) if len(extrinsic_raw) == 6 else np.array(extrinsic_raw).reshape(4, 4)

            if model_type == "vadas":
                camera_model = VADASFisheyeCameraModel(intrinsic, image_size=image_size)
            else:
                raise ValueError(f"Unsupported camera model: {model_type}.")
            
            self.sensors[cam_name] = SensorInfo(cam_name, camera_model, intrinsic, extrinsic_matrix, image_size)

    def _rodrigues_to_matrix(self, rvec_tvec: List[float]) -> np.ndarray:
        tvec = np.array(rvec_tvec[0:3]).reshape(3, 1)
        rvec = np.array(rvec_tvec[3:6])
        R, _ = cv2.Rodrigues(rvec)
        
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 0:3] = R
        transform_matrix[0:3, 3:4] = tvec
        return transform_matrix

    def get(self, name: str) -> SensorInfo:
        if name not in self.sensors:
            raise ValueError(f"Sensor '{name}' not found in calibration database.")
        return self.sensors[name]

def load_pcd_xyz(path: Path) -> np.ndarray:
    if OPEN3D_AVAILABLE:
        try:
            pcd = o3d.io.read_point_cloud(str(path))
            return np.asarray(pcd.points, dtype=np.float64) if pcd.has_points() else np.empty((0, 3))
        except Exception:
            pass
    
    points = []
    with open(path, 'r', encoding='utf-8') as f:
        data_started = False
        for line in f:
            if data_started:
                try:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                except (ValueError, IndexError):
                    continue
            elif line.startswith("DATA ascii"):
                data_started = True
    return np.array(points, dtype=np.float64)

class LidarCameraProjector:
    """Projects LiDAR point clouds to create depth maps."""
    def __init__(self, calib_db: CalibrationDB):
        self.calib_db = calib_db

    def project_cloud_to_depth_map(self, sensor_name: str, cloud_xyz: np.ndarray, image_size: Tuple[int, int]) -> Optional[np.ndarray]:
        sensor_info = self.calib_db.get(sensor_name)
        camera_model = sensor_info.model
        cam_extrinsic = sensor_info.extrinsic
        image_width, image_height = image_size

        if isinstance(camera_model, VADASFisheyeCameraModel) and camera_model.image_size is None:
            camera_model.image_size = (image_width, image_height)

        depth_map = np.zeros((image_height, image_width), dtype=np.float32)
        
        cloud_xyz_hom = np.hstack((cloud_xyz, np.ones((cloud_xyz.shape[0], 1))))
        
        lidar_to_camera_transform = np.linalg.inv(cam_extrinsic)
        points_cam_hom = (lidar_to_camera_transform @ cloud_xyz_hom.T).T
        points_cam = points_cam_hom[:, :3]

        for i in range(points_cam.shape[0]):
            Xc, Yc, Zc = points_cam[i]
            
            if Xc <= 0:
                continue

            u, v, valid_projection = camera_model.project_point(Xc, Yc, Zc)

            if valid_projection and 0 <= u < image_width and 0 <= v < image_height:
                if depth_map[v, u] == 0 or depth_map[v, u] > Xc:
                    depth_map[v, u] = Xc
        
        return depth_map

def save_depth_map(path: Path, depth_map: np.ndarray):
    """Saves a depth map as a 16-bit PNG image."""
    depth_map_uint16 = (depth_map * 256.0).astype(np.uint16)
    cv2.imwrite(str(path), depth_map_uint16)

# --- 기본 캘리브레이션 데이터 ---
DEFAULT_CALIB = {
  "a6": {
    "model": "vadas",
    "intrinsic": [-0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,
                  1.0447, 0.0021, 44.9516, 2.48822, 0, 0.9965, -0.0067,
                  -0.0956, 0.1006, -0.054, 0.0106],
    "extrinsic": [ 0.293769, -0.0542026, -0.631615, -0.00394431, -0.33116, -0.00963617],
    "image_size": None
  }
}

def main():
    """
    '0401_calib_data' 폴더의 PCD와 PNG 파일을 사용하여 Depth Map을 생성합니다.
    """
    input_folder = Path("./0401_calib_data")
    output_folder = Path("./output/depth_maps_0401")
    output_folder.mkdir(parents=True, exist_ok=True)

    print(f"입력 폴더: {input_folder.resolve()}")
    print(f"출력 폴더: {output_folder.resolve()}")

    try:
        calib_db = CalibrationDB(DEFAULT_CALIB)
        projector = LidarCameraProjector(calib_db)
        
        pcd_files = sorted(list(input_folder.glob("*.pcd")))
        if not pcd_files:
            print(f"오류: '{input_folder}' 폴더에서 PCD 파일을 찾을 수 없습니다.", file=sys.stderr)
            return

        print(f"{len(pcd_files)}개의 PCD 파일을 처리합니다...")

        for pcd_path in tqdm(pcd_files):
            image_path = pcd_path.with_suffix(".png")
            if not image_path.exists():
                print(f"경고: {pcd_path.name}에 해당하는 PNG 파일({image_path.name})을 찾을 수 없어 건너<binary data, 1 bytes>니다.", file=sys.stderr)
                continue

            try:
                # 이미지 로드하여 해상도 정보 얻기
                image = cv2.imread(str(image_path))
                if image is None:
                    raise IOError(f"이미지 파일을 로드할 수 없습니다: {image_path}")
                image_height, image_width, _ = image.shape

                # PCD 파일 로드
                cloud_xyz = load_pcd_xyz(pcd_path)
                if cloud_xyz.shape[0] == 0:
                    print(f"경고: {pcd_path.name}에 포인트가 없어 건너<binary data, 1 bytes>니다.", file=sys.stderr)
                    continue

                # Depth Map 생성
                depth_map = projector.project_cloud_to_depth_map("a6", cloud_xyz, (image_width, image_height))

                if depth_map is not None:
                    output_path = output_folder / f"{pcd_path.stem}.png"
                    save_depth_map(output_path, depth_map)

            except Exception as e:
                print(f"'{pcd_path.name}' 처리 중 오류 발생: {e}", file=sys.stderr)

        print("Depth map 생성이 완료되었습니다.")

    except Exception as e:
        print(f"스크립트 실행 중 오류 발생: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
