"""
PCD 파일만 불러와서 Open3D로 시각화하는 심플한 도구
"""

import argparse
import numpy as np
from pathlib import Path

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("⚠️ Open3D가 설치되지 않았습니다. pip install open3d")


def load_pcd_xyz(pcd_path: Path) -> np.ndarray:
    """PCD 파일에서 XYZ 좌표만 추출 (Binary/ASCII 모두 지원)"""
    with open(pcd_path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header를 읽는 중 파일이 끝났습니다")
            decoded = line.decode('utf-8', errors='ignore').strip()
            header_lines.append(decoded)
            if decoded.startswith('DATA'):
                break

        num_points = 0
        data_format = 'ascii'
        fields = []
        sizes = []
        types = []
        counts = []

        for line in header_lines:
            tokens = line.split()
            if not tokens:
                continue
            key = tokens[0].upper()
            if key == 'POINTS':
                num_points = int(tokens[1])
            elif key == 'DATA':
                data_format = tokens[1]
            elif key == 'FIELDS':
                fields = tokens[1:]
            elif key == 'SIZE':
                sizes = list(map(int, tokens[1:]))
            elif key == 'TYPE':
                types = tokens[1:]
            elif key == 'COUNT':
                counts = list(map(int, tokens[1:]))

        if not counts and fields:
            counts = [1] * len(fields)

        print(f"  포인트 수: {num_points}")
        print(f"  데이터 포맷: {data_format}")
        print(f"  필드: {fields}")

        if data_format == 'binary':
            dtype_fields = []
            for field_name, size, typ, cnt in zip(fields, sizes, types, counts):
                if typ == 'F':
                    dtype = np.float32 if size == 4 else np.float64
                elif typ == 'U':
                    dtype = np.uint8 if size == 1 else (np.uint16 if size == 2 else np.uint32)
                elif typ == 'I':
                    dtype = np.int8 if size == 1 else (np.int16 if size == 2 else np.int32)
                else:
                    raise ValueError(f"지원되지 않는 TYPE: {typ}")

                if cnt == 1:
                    dtype_fields.append((field_name, dtype))
                else:
                    dtype_fields.append((field_name, (dtype, cnt)))

            dtype = np.dtype(dtype_fields)
            data = np.fromfile(f, dtype=dtype, count=num_points)
            if {'x', 'y', 'z'} - set(dtype.names or []):
                raise ValueError("PCD 데이터에 x/y/z 필드가 없습니다")
            xyz = np.stack([data['x'], data['y'], data['z']], axis=-1).astype(np.float32)
            return xyz

        elif data_format == 'binary_compressed':
            raise NotImplementedError("binary_compressed PCD는 아직 지원되지 않습니다")

        # ASCII fallback
        points_list = []
        for line in f:
            parts = line.decode('utf-8', errors='ignore').strip().split()
            if len(parts) >= 3:
                points_list.append([float(parts[0]), float(parts[1]), float(parts[2])])
        return np.array(points_list, dtype=np.float32)


def visualize_pcd(pcd_path: Path):
    """Open3D로 PCD 파일을 3D 시각화"""
    if not HAS_OPEN3D:
        print("❌ Open3D가 필요합니다")
        return
    
    print(f"\n[1] PCD 파일 로드")
    print(f"  경로: {pcd_path}")
    
    # XYZ 좌표 로드
    cloud_xyz = load_pcd_xyz(pcd_path)
    print(f"  ✓ 로드 완료: {cloud_xyz.shape[0]} 포인트")
    
    # 기본 통계
    print(f"\n[2] 포인트 통계")
    print(f"  X: min={cloud_xyz[:,0].min():.2f}, max={cloud_xyz[:,0].max():.2f}")
    print(f"  Y: min={cloud_xyz[:,1].min():.2f}, max={cloud_xyz[:,1].max():.2f}")
    print(f"  Z: min={cloud_xyz[:,2].min():.2f}, max={cloud_xyz[:,2].max():.2f}")
    
    # Open3D 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_xyz)
    
    # 좌표축 추가
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=5.0, origin=[0, 0, 0]
    )
    
    print(f"\n[3] 3D 시각화")
    print("  좌표축 색상:")
    print("    🔴 빨간색 = X축")
    print("    🟢 초록색 = Y축")
    print("    🔵 파란색 = Z축")
    
    # 시각화
    o3d.visualization.draw_geometries(
        [pcd, coord_frame],
        window_name=f"PCD Viewer: {pcd_path.name}",
        width=1200,
        height=800
    )


def main():
    parser = argparse.ArgumentParser(description="Simple PCD Viewer")
    parser.add_argument(
        "--pcd",
        type=str,
        default="synchronized_data_pangyo_optimized/pcd/0000050000.pcd",
        help="PCD 파일 경로",
    )
    args = parser.parse_args()
    
    pcd_path = Path(args.pcd)
    
    if not pcd_path.exists():
        print(f"❌ 파일을 찾을 수 없습니다: {pcd_path}")
        return
    
    visualize_pcd(pcd_path)


if __name__ == "__main__":
    main()
