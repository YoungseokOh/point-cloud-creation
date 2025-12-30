#!/usr/bin/env python3
"""
Run indoor v4 projection for multiple indoor datasets and save grids.
Datasets (default):
- synchronized_data_indoor_50ms
- synchronized_data_indoor_100ms
- synchronized_data_indoor_150ms

Usage examples:
    python indoor_verification_loop.py --frames 0000020100,0000020228
    python indoor_verification_loop.py --random 3
    python indoor_verification_loop.py --datasets synchronized_data_indoor_50ms,synchronized_data_indoor_150ms
"""
import argparse
import math
import random
import sys
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np

# =============================================================================
# Calibration (borrowed from indoor_verification.py - computed LiDAR→Camera)
# =============================================================================
DEFAULT_CALIB_v4 = {
    "a6": {
        "model": "vadas",
        "intrinsic": [
            -0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,  # u2d (k[0:7])
            1.05007, 0.0021, -0.00152588, 0.000793457,  # s, div, ux, uy (updated)
            0, 0.9965, -0.0067, -0.0956, 0.1006, -0.054, 0.0106  # d2u
        ],
        "extrinsic": [0.00660233, 0.0122275, -0.230106, -0.0429939, 0.0809541, 0.01427919],
        "image_size": (1920, 1536)
    }
}

DEFAULT_LIDAR_TO_WORLD_v4 = np.array([
       [0.999823, -0.0183399, -0.00420554, -0.0510422],
       [0.0182813, 0.999741, -0.0135742, -0.0243783],
       [0.0044534, 0.013495, 0.999899, 0.341502],
       [0, 0, 0, 1]
])


def parse_pcd_file(pcd_path: Path) -> np.ndarray:
    """Parse PCD (binary or ASCII) -> Nx3 array."""
    try:
        with open(pcd_path, 'rb') as f:
            header_lines = []
            while True:
                line = f.readline()
                try:
                    line_str = line.decode('ascii').strip()
                except Exception:
                    break
                header_lines.append(line_str)
                if line_str.startswith('DATA'):
                    break
            is_binary = False
            num_points = 0
            fields = []
            sizes = []
            types = []
            for line in header_lines:
                if line.startswith('POINTS'):
                    num_points = int(line.split()[1])
                elif line.startswith('FIELDS'):
                    fields = line.split()[1:]
                elif line.startswith('SIZE'):
                    sizes = [int(x) for x in line.split()[1:]]
                elif line.startswith('TYPE'):
                    types = line.split()[1:]
                elif line.startswith('DATA'):
                    if 'binary' in line.lower():
                        is_binary = True
            if is_binary:
                dtype_list = []
                for field, size, typ in zip(fields, sizes, types):
                    if typ == 'F':
                        dtype_list.append((field, np.float32 if size == 4 else np.float64))
                    elif typ == 'U':
                        if size == 4:
                            dtype_list.append((field, np.uint32))
                        elif size == 2:
                            dtype_list.append((field, np.uint16))
                        else:
                            dtype_list.append((field, np.uint8))
                    elif typ == 'I':
                        dtype_list.append((field, np.int32 if size == 4 else np.int16))
                dtype = np.dtype(dtype_list)
                data = np.frombuffer(f.read(), dtype=dtype, count=num_points)
                return np.column_stack([data['x'], data['y'], data['z']])
    except Exception:
        pass

    # ASCII fallback
    points = []
    try:
        with open(pcd_path, 'r', encoding='utf-8') as f:
            in_data = False
            for line in f:
                line = line.strip()
                if line.startswith('DATA'):
                    in_data = True
                    continue
                if in_data and line:
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                        except ValueError:
                            continue
    except Exception:
        return np.zeros((0, 3))
    return np.array(points) if points else np.zeros((0, 3))


def rodrigues_to_matrix(rodrigues_vec):
    tvec = np.array(rodrigues_vec[:3], dtype=np.float64)
    rvec = np.array(rodrigues_vec[3:6], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


class VADASFisheyeProjector:
    def __init__(self, intrinsic, extrinsic, lidar_to_world, image_size):
        self.k = intrinsic[0:7]
        self.s = intrinsic[7]
        self.div = intrinsic[8]
        self.ux = intrinsic[9]
        self.uy = intrinsic[10]
        self.image_size = image_size

        self.world_to_camera = rodrigues_to_matrix(extrinsic)
        self.lidar_to_world = lidar_to_world
        self.lidar_to_camera = self.world_to_camera @ self.lidar_to_world

    def _poly_eval(self, coeffs, x):
        result = 0.0
        for c in reversed(coeffs):
            result = result * x + c
        return result

    def project_point(self, Xc, Yc, Zc):
        nx = -Yc
        ny = -Zc
        dist = math.hypot(nx, ny)
        if dist < sys.float_info.epsilon:
            return None, None, False
        cosPhi = nx / dist
        sinPhi = ny / dist
        theta = math.atan2(dist, Xc)
        if Xc < 0:
            return None, None, False
        xd = theta * self.s
        if abs(self.div) < 1e-9:
            return None, None, False
        rd = self._poly_eval(self.k, xd) / self.div
        if math.isinf(rd) or math.isnan(rd):
            return None, None, False
        img_w_half = self.image_size[0] / 2
        img_h_half = self.image_size[1] / 2
        u = rd * cosPhi + self.ux + img_w_half
        v = rd * sinPhi + self.uy + img_h_half
        if 0 <= u < self.image_size[0] and 0 <= v < self.image_size[1]:
            return int(u), int(v), True
        return None, None, False

    def project_points(self, points_lidar):
        if points_lidar.size == 0:
            return [], [], []
        points_hom = np.hstack([points_lidar, np.ones((points_lidar.shape[0], 1))])
        points_cam = (self.lidar_to_camera @ points_hom.T).T[:, :3]
        us, vs, depths = [], [], []
        for Xc, Yc, Zc in points_cam:
            u, v, valid = self.project_point(Xc, Yc, Zc)
            if valid:
                us.append(u)
                vs.append(v)
                depths.append(Xc)
        return us, vs, depths


def create_depth_overlay(rgb_image, us, vs, depths, point_size=3, max_depth=15.0):
    overlay = rgb_image.copy()
    if len(us) == 0:
        return overlay
    depths_arr = np.array(depths)
    depths_norm = np.clip(depths_arr / max_depth, 0, 1)
    depths_uint8 = (depths_norm * 255).astype(np.uint8)
    colors = cv2.applyColorMap(depths_uint8.reshape(-1, 1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
    for u, v, c in zip(us, vs, colors):
        cv2.circle(overlay, (u, v), point_size, c.tolist(), -1)
    return overlay


def zero_pad_frame(fid: str) -> str:
    return fid if not fid.isdigit() else fid.zfill(10)


def find_image_path(dataset_dir: Path, frame_id: str) -> Optional[Path]:
    candidates = [dataset_dir / "image_a6" / f"{frame_id}.jpg",
                  dataset_dir / "image_a6" / f"{frame_id}.png",
                  dataset_dir / "img" / f"{frame_id}.jpg",
                  dataset_dir / "img" / f"{frame_id}.png"]
    for p in candidates:
        if p.exists():
            return p
    return None


def find_pcd_path(dataset_dir: Path, frame_id: str) -> Optional[Path]:
    candidates = [dataset_dir / "pcd" / f"{frame_id}.pcd",
                  dataset_dir / "pcd" / f"{frame_id}.bin"]
    for p in candidates:
        if p.exists():
            return p
    return None


def verify_frame_in_dataset(dataset_dir: Path, frame_id: str, projector: VADASFisheyeProjector,
                             point_size: int, max_depth: float):
    img_path = find_image_path(dataset_dir, frame_id)
    pcd_path = find_pcd_path(dataset_dir, frame_id)
    if img_path is None or pcd_path is None:
        print(f"[WARN] Missing img/pcd for frame {frame_id} in {dataset_dir.name}")
        return None
    rgb_image = cv2.imread(str(img_path))
    if rgb_image is None:
        print(f"[WARN] Failed to load image: {img_path}")
        return None
    points = parse_pcd_file(pcd_path)
    forward_mask = points[:, 0] > 0
    forward_points = points[forward_mask]
    us, vs, depths = projector.project_points(forward_points)
    overlay = create_depth_overlay(rgb_image, us, vs, depths, point_size=point_size, max_depth=max_depth)
    return overlay, len(us), len(forward_points)


def create_grid(results, output_path: Path, cell_w=640, cell_h=512):
    if not results:
        return
    cols = min(3, len(results))
    rows = (len(results) + cols - 1) // cols
    legend_h = 80
    grid = np.zeros((rows * cell_h + legend_h, cols * cell_w, 3), dtype=np.uint8)
    grid[:] = (40, 40, 40)
    for i, (fid, overlay, num_pts, total_pts) in enumerate(results):
        r, c = divmod(i, cols)
        x0, y0 = c * cell_w, r * cell_h
        resized = cv2.resize(overlay, (cell_w, cell_h))
        cv2.rectangle(resized, (0, 0), (cell_w, 50), (0, 0, 0), -1)
        cv2.putText(resized, f"{fid}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(resized, f"pts: {num_pts}/{total_pts}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        grid[y0:y0+cell_h, x0:x0+cell_w] = resized
    cv2.putText(grid, "Indoor v4 projection", (20, rows * cell_h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imwrite(str(output_path), grid)
    print(f"[SAVE] Grid -> {output_path}")


def process_dataset(dataset_dir: Path, frame_ids: List[str], output_dir: Path,
                    point_size: int, max_depth: float):
    calib = DEFAULT_CALIB_v4["a6"]
    projector = VADASFisheyeProjector(
        intrinsic=calib["intrinsic"],
        extrinsic=calib["extrinsic"],
        lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v4,
        image_size=calib["image_size"]
    )
    results = []
    output_dir.mkdir(parents=True, exist_ok=True)
    for fid_raw in frame_ids:
        fid = zero_pad_frame(fid_raw)
        res = verify_frame_in_dataset(dataset_dir, fid, projector, point_size, max_depth)
        if res is None:
            continue
        overlay, num_pts, total_pts = res
        cv2.imwrite(str(output_dir / f"{fid}_proj.jpg"), overlay)
        results.append((fid, overlay, num_pts, total_pts))
    if len(results) > 1:
        create_grid(results, output_dir / "grid.jpg")


def collect_frames(dataset_dir: Path, frames_arg: Optional[str], random_k: int, seed: int) -> List[str]:
    if frames_arg:
        return [f.strip() for f in frames_arg.split(',') if f.strip()]
    pcd_dir = dataset_dir / "pcd"
    pcd_files = sorted(pcd_dir.glob("*.pcd"))
    if not pcd_files:
        return []
    if random_k < 0:
        # process all
        return [p.stem for p in pcd_files]
    random.seed(seed)
    k = min(random_k, len(pcd_files))
    return [p.stem for p in random.sample(pcd_files, k)]


def main():
    parser = argparse.ArgumentParser(description="Indoor v4 projection loop for multiple datasets")
    parser.add_argument("--data_root", type=str, default=r"D:\data\ncdb-cls\ncdb-cls-indoor",
                        help="Root directory containing indoor datasets")
    parser.add_argument("--datasets", type=str,
                        default="synchronized_data_indoor_50ms_second,synchronized_data_indoor_100ms_second,synchronized_data_indoor_150ms_second",
                        help="Comma-separated dataset folder names under data_root")
    parser.add_argument("--frames", type=str, default=None,
                        help="Comma-separated frame IDs to project; if omitted, uses --random")
    parser.add_argument("--random", type=int, default=3,
                        help="Number of random frames per dataset when --frames is not provided")
    parser.add_argument("--all", action="store_true",
                        help="Process all frames in each dataset (overrides --random)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--output_dir", type=str, default="./output/indoor_loop",
                        help="Output base directory")
    parser.add_argument("--point_size", type=int, default=3, help="Rendered point size")
    parser.add_argument("--max_depth", type=float, default=15.0, help="Max depth for colormap normalization")
    args = parser.parse_args()

    data_root = Path(args.data_root)
    dataset_names = [d.strip() for d in args.datasets.split(',') if d.strip()]
    for ds_name in dataset_names:
        ds_dir = data_root / ds_name
        if not ds_dir.exists():
            print(f"[WARN] Dataset not found: {ds_dir}")
            continue
        rand_k = -1 if args.all else args.random
        frame_ids = collect_frames(ds_dir, args.frames, rand_k, args.seed)
        if not frame_ids:
            print(f"[WARN] No frames found in {ds_dir}")
            continue
        print(f"\n=== Processing {ds_name} ({len(frame_ids)} frames) ===")
        out_dir = Path(args.output_dir) / ds_name
        process_dataset(ds_dir, frame_ids, out_dir, args.point_size, args.max_depth)

    print("\nDone.")


if __name__ == "__main__":
    main()
