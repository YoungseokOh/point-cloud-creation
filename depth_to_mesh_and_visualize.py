import numpy as np
import open3d as o3d
import cv2
from pathlib import Path

def depth_to_point_cloud(depth_map, fx, fy, cx, cy, depth_scale=1.0):
    """
    깊이 맵과 카메라 내부 파라미터를 사용하여 3D 포인트 클라우드를 생성합니다.
    Pinhole 카메라 모델을 가정합니다.
    """
    H, W = depth_map.shape
    
    # 픽셀 좌표 그리드 생성
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    
    # 깊이 맵의 유효한 값만 사용 (0이 아닌 값)
    # Depth Anything V2의 출력은 0이 아닌 경우가 많으므로, 특정 임계값 이상만 유효하다고 판단할 수 있습니다.
    # 여기서는 0.1m 이상을 유효한 깊이로 간주합니다.
    valid_mask = depth_map > 0.1
    
    # 유효한 픽셀에 대해서만 계산
    u_valid = u[valid_mask]
    v_valid = v[valid_mask]
    depth_valid = depth_map[valid_mask] * depth_scale # 스케일 적용
    
    # 3D 포인트 계산
    x_coords = (u_valid - cx) * depth_valid / fx
    y_coords = (v_valid - cy) * depth_valid / fy
    z_coords = depth_valid
    
    points = np.vstack((x_coords, y_coords, z_coords)).T
    
    return points

def create_mesh_from_depth_map(depth_npy_path, output_mesh_path, 
                                fx, fy, cx, cy,
                                depth_scale=1.0, depth_min_clip=0.1, depth_max_clip=30.0,
                                poisson_depth=9):
    """
    Depth Anything V2의 .raw.npy 깊이 맵을 사용하여 Mesh를 생성합니다.
    """
    print(f"[INFO] 깊이 맵 로드 중: {depth_npy_path}")
    depth_map_raw = np.load(depth_npy_path)
    
    # Depth Anything V2의 raw 출력은 일반적으로 inverse depth 또는 유사한 형태입니다.
    # analyze_distribution.py에서 np.max(da_raw) - da_raw 로 반전하는 것을 보았으므로,
    # 여기서는 이 과정을 적용하여 '가까운 것이 작은 값, 먼 것이 큰 값'으로 만듭니다.
    depth_map_inverted = np.max(depth_map_raw) - depth_map_raw
    
    # 깊이 값 클리핑 (너무 가깝거나 너무 먼 값 제거)
    depth_map_clipped = np.clip(depth_map_inverted, depth_min_clip, depth_max_clip)
    
    H, W = depth_map_clipped.shape
    
    print(f"[INFO] 카메라 파라미터: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
    print(f"[INFO] 깊이 맵 해상도: {W}x{H}")

    # 3D 포인트 클라우드 생성
    print("[INFO] 3D 포인트 클라우드 생성 중...")
    points = depth_to_point_cloud(depth_map_clipped, fx, fy, cx, cy, depth_scale)
    
    if points.shape[0] == 0:
        print("[ERROR] 유효한 3D 포인트가 생성되지 않았습니다. 깊이 맵을 확인하세요.")
        return None

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 노멀 추정
    print("[INFO] 포인트 클라우드 노멀 추정 중...")
    # 노멀 추정 파라미터 조정: radius를 포인트 클라우드의 밀도에 맞게 조정
    # 여기서는 임의로 0.1로 설정했지만, 실제 데이터에 따라 조정 필요
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.orient_normals_towards_camera_location(pcd.get_center()) # 카메라 방향으로 노멀 정렬

    # Poisson Surface Reconstruction
    print(f"[INFO] Poisson Surface Reconstruction 수행 중 (depth={poisson_depth})...")
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=poisson_depth, scale=1.1, linear_fit=False
        )
    
    # 밀도가 낮은 부분 제거 (선택 사항)
    # bbox = pcd.get_axis_aligned_bounding_box()
    # mesh = mesh.crop(bbox)

    # Mesh 저장
    print(f"[INFO] Mesh 저장 중: {output_mesh_path}")
    o3d.io.write_triangle_mesh(str(output_mesh_path), mesh)
    print("[INFO] Mesh 생성 및 저장 완료.")
    
    return mesh

def visualize_mesh_and_image(mesh, image_path, image_width, image_height):
    """
    생성된 Mesh와 원본 이미지를 함께 시각화합니다.
    Mesh는 흑백 음영으로 표시됩니다.
    """
    print(f"[INFO] 시각화 시작. 이미지: {image_path}")
    
    # 이미지 로드
    image = cv2.imread(str(image_path))
    if image is None:
        print(f"[ERROR] 이미지 로드 실패: {image_path}")
        return

    # Open3D 시각화 객체 생성
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Mesh and Image Visualization", width=image_width, height=image_height)

    # Mesh에 흑백 음영 적용
    # Mesh의 노멀을 기반으로 색상을 설정하여 음영 효과를 줍니다.
    # 여기서는 간단히 노멀의 Z축 성분을 사용하여 깊이감을 표현합니다.
    # 더 복잡한 조명 모델을 적용할 수도 있습니다.
    if len(mesh.normals) > 0: # mesh.has_normals() 대신 len(mesh.normals) > 0 사용
        # 더 간단한 흑백 음영: 모든 정점 색상을 회색으로 설정
        mesh.paint_uniform_color([0.5, 0.5, 0.5]) # 기본 회색
        # 조명 설정은 뷰어에서 자동으로 처리되므로, Mesh 자체의 색상은 균일하게 유지하고
        # 뷰어의 조명 설정을 통해 음영을 표현하는 것이 일반적입니다.

    vis.add_geometry(mesh)

    # 뷰포트 설정
    view_control = vis.get_view_control()
    # 카메라 위치 및 방향 조정 (Mesh가 더 잘 보이도록)
    # 이 값들은 Mesh의 스케일과 형태에 따라 조정이 필요합니다.
    view_control.set_front([0, 0, -1]) # 카메라가 Z축 음의 방향을 보도록
    view_control.set_up([0, -1, 0])    # Y축이 위로 향하도록
    view_control.set_zoom(0.5)         # 초기 줌 레벨
    view_control.set_lookat([0, 0, 5]) # 카메라가 바라보는 지점 (Mesh의 대략적인 중앙)

    vis.run()
    vis.destroy_window()
    print("[INFO] 시각화 종료.")

    # 원본 이미지를 별도의 창으로 보여주기
    cv2.imshow("Original Raw Depth Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # --- 사용자 설정 ---
    depth_npy_path = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/output/fisheye_da_931_results/output.raw.npy")
    raw_png_path = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/output/fisheye_da_931_results/output.raw.png")
    
    output_dir = Path("C:/Users/seok436/Documents/VSCode/Projects/point-cloud-creation/point-cloud-creation/output/generated_mesh")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_mesh_path = output_dir / "depth_anything_mesh.ply"

    # 깊이 맵의 실제 해상도를 로드하여 사용
    try:
        temp_depth_map = np.load(depth_npy_path)
        H_img, W_img = temp_depth_map.shape
        del temp_depth_map # 메모리 해제
    except Exception as e:
        print(f"[ERROR] 깊이 맵 해상도 로드 실패: {e}. 기본값 (1920x1536)으로 cx, cy 설정.")
        W_img, H_img = 1920, 1536 # Fallback to user-provided size

    camera_cx = W_img / 2.0
    camera_cy = H_img / 2.0

    # fx, fy는 임의의 값으로, 실제 카메라의 FOV에 따라 달라집니다.
    # 이 값을 조정하여 3D 모델의 "깊이감"이나 "스케일"을 조절할 수 있습니다.
    # 1920x1536 해상도에 맞춰 FOV를 적절히 조절하기 위해 fx, fy를 조정합니다.
    # 여기서는 일단 1000으로 설정하여 이전보다 넓은 FOV를 가정합니다.
    camera_fx = 1000.0 # 조정된 초점 거리
    camera_fy = 1000.0 # 조정된 초점 거리

    # Depth Anything V2의 출력 스케일 조정 (임의의 값, 실제 스케일은 analyze_distribution.py의 결과 참고)
    # analyze_distribution.py에서 GT와 매칭된 깊이 맵이 256.0으로 나누어 미터 단위로 변환되었으므로,
    # raw 출력에 대한 스케일도 유사하게 조정할 필요가 있습니다.
    # 여기서는 임의로 1.0으로 설정하고, 필요시 analyze_distribution.py의 스케일링 팩터를 적용할 수 있습니다.
    depth_scale_factor = 1.0 # 실제 미터 단위로 변환하기 위한 스케일 팩터 (조정 필요)

    # 깊이 클리핑 범위 (analyze_distribution.py의 matching_threshold_m 참고)
    depth_min_clip_m = 0.1
    depth_max_clip_m = 30.0 # 30m로 설정 (analyze_distribution.py의 25m와 유사하게)

    # Poisson Reconstruction의 깊이 파라미터 (높을수록 더 세밀한 Mesh)
    poisson_reconstruction_depth = 9 

    # Mesh 생성
    generated_mesh = create_mesh_from_depth_map(
        depth_npy_path, output_mesh_path,
        fx=camera_fx, fy=camera_fy, cx=camera_cx, cy=camera_cy,
        depth_scale=depth_scale_factor, depth_min_clip=depth_min_clip_m, depth_max_clip=depth_max_clip_m,
        poisson_depth=poisson_reconstruction_depth
    )

    if generated_mesh:
        # Mesh와 원본 이미지 시각화
        visualize_mesh_and_image(generated_mesh, raw_png_path, W_img, H_img)
    else:
        print("[ERROR] Mesh 생성에 실패하여 시각화를 건너뛰었습니다.")
