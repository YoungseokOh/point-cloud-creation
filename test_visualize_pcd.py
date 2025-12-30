#!/usr/bin/env python3
"""
visualize_pcd.py 테스트 및 데모 스크립트

이 스크립트는 visualize_pcd.py의 기능을 테스트하고 데모합니다.
"""

import sys
from pathlib import Path
import numpy as np
import cv2

# visualize_pcd 모듈 임포트
try:
    from visualize_pcd import (
        load_pcd_xyz,
        project_cloud_to_image,
        CalibrationDB,
        VADASFisheyeCameraModel,
        DEFAULT_CALIB,
        DEFAULT_LIDAR_TO_WORLD_v3,
        visualize_pcd_3d,
        get_color_from_distance
    )
except ImportError as e:
    print(f"Error: visualize_pcd 모듈을 임포트할 수 없습니다: {e}")
    sys.exit(1)


def test_vadas_camera_model():
    """VADAS 카메라 모델 테스트"""
    print("\n" + "="*70)
    print("테스트 1: VADAS 카메라 모델")
    print("="*70)
    
    # 카메라 모델 생성
    camera = VADASFisheyeCameraModel(
        intrinsic=DEFAULT_CALIB["a6"]["intrinsic"],
        image_size=(1920, 1536)
    )
    
    print(f"✓ 카메라 모델 생성")
    print(f"  이미지 크기: {camera.image_size}")
    print(f"  Intrinsic: {camera.intrinsic[:3]}...")
    
    # 투영 테스트
    test_points = [
        (1.0, 0.0, 0.0),      # 카메라 중앙 앞
        (1.0, 0.5, 0.0),      # 오른쪽
        (1.0, 0.0, 0.5),      # 아래
        (5.0, 1.0, 1.0),      # 먼 포인트
        (1.0, 0.0, -1.0),     # 뒤쪽 (범위 밖)
    ]
    
    print(f"\n투영 테스트:")
    print(f"  {'Xc':<8} {'Yc':<8} {'Zc':<8} {'u':<8} {'v':<8} {'유효':<8}")
    print(f"  {'-'*50}")
    
    for Xc, Yc, Zc in test_points:
        u, v, valid = camera.project_point(Xc, Yc, Zc)
        print(f"  {Xc:<8.2f} {Yc:<8.2f} {Zc:<8.2f} {u:<8} {v:<8} {'O' if valid else 'X':<8}")


def test_calibration_db():
    """보정 데이터베이스 테스트"""
    print("\n" + "="*70)
    print("테스트 2: 보정 데이터베이스")
    print("="*70)
    
    # CalibrationDB 생성
    calib_db = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v3)
    
    print(f"✓ CalibrationDB 생성")
    print(f"  LiDAR to World 행렬:")
    print(f"    {DEFAULT_LIDAR_TO_WORLD_v3[0]}")
    print(f"    {DEFAULT_LIDAR_TO_WORLD_v3[1]}")
    print(f"    {DEFAULT_LIDAR_TO_WORLD_v3[2]}")
    print(f"    {DEFAULT_LIDAR_TO_WORLD_v3[3]}")
    
    # 카메라 정보 조회
    camera_info = calib_db.get("a6")
    print(f"\n✓ 카메라 정보 (a6):")
    print(f"  Extrinsic 행렬:")
    print(f"    {camera_info['extrinsic'][0]}")
    print(f"    {camera_info['extrinsic'][1]}")
    print(f"    {camera_info['extrinsic'][2]}")
    print(f"    {camera_info['extrinsic'][3]}")


def test_pcd_loading():
    """PCD 파일 로딩 테스트"""
    print("\n" + "="*70)
    print("테스트 3: PCD 파일 로딩")
    print("="*70)
    
    pcd_path = Path("ncdb-cls-sample/synced_data/pcd/0000000931.pcd")
    
    if not pcd_path.exists():
        print(f"⚠ PCD 파일을 찾을 수 없습니다: {pcd_path}")
        print(f"  데이터 경로를 확인하세요")
        return False
    
    try:
        cloud_xyz = load_pcd_xyz(pcd_path)
        
        print(f"✓ PCD 파일 로드 완료: {pcd_path.name}")
        print(f"  포인트 수: {len(cloud_xyz)}")
        print(f"  X 범위: {cloud_xyz[:, 0].min():.2f} ~ {cloud_xyz[:, 0].max():.2f}")
        print(f"  Y 범위: {cloud_xyz[:, 1].min():.2f} ~ {cloud_xyz[:, 1].max():.2f}")
        print(f"  Z 범위: {cloud_xyz[:, 2].min():.2f} ~ {cloud_xyz[:, 2].max():.2f}")
        
        return True
    except Exception as e:
        print(f"❌ Error: {e}")
        return False


def test_projection():
    """투영 테스트"""
    print("\n" + "="*70)
    print("테스트 4: PCD를 이미지에 투영")
    print("="*70)
    
    # 경로 설정
    pcd_path = Path("ncdb-cls-sample/synced_data/pcd/0000000931.pcd")
    image_path = Path("ncdb-cls-sample/synced_data/image_a6/0000000931.jpg")
    output_path = Path("output/test_projection.jpg")
    
    # 파일 존재 확인
    if not pcd_path.exists():
        print(f"⚠ PCD 파일을 찾을 수 없습니다: {pcd_path}")
        return False
    
    if not image_path.exists():
        print(f"⚠ 이미지 파일을 찾을 수 없습니다: {image_path}")
        return False
    
    try:
        # PCD 로드
        print(f"PCD 로드 중: {pcd_path.name}...")
        cloud_xyz = load_pcd_xyz(pcd_path)
        
        # 이미지 로드
        print(f"이미지 로드 중: {image_path.name}...")
        image = cv2.imread(str(image_path))
        h, w = image.shape[:2]
        
        # 보정 데이터 설정
        print(f"보정 데이터 설정 중...")
        calib_db = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v3)
        
        # 투영
        print(f"투영 중...")
        output_image, in_front, on_image = project_cloud_to_image(
            cloud_xyz=cloud_xyz,
            image=image,
            calib_db=calib_db,
            camera_name="a6",
            max_distance=50.0,
            point_radius=2
        )
        
        # 결과 저장
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), output_image)
        
        # 통계
        print(f"\n✓ 투영 완료")
        print(f"  총 포인트: {len(cloud_xyz)}")
        print(f"  카메라 앞: {in_front} ({100*in_front/len(cloud_xyz):.1f}%)")
        print(f"  이미지 범위: {on_image} ({100*on_image/len(cloud_xyz):.1f}%)")
        print(f"  결과 저장: {output_path}")
        
        return True
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_color_mapping():
    """색상 매핑 테스트"""
    print("\n" + "="*70)
    print("테스트 5: 거리별 색상 매핑")
    print("="*70)
    
    distances = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
    max_distance = 50.0
    
    print(f"거리별 색상 (BGR):")
    print(f"  {'거리(m)':<10} {'B':<8} {'G':<8} {'R':<8} {'설명':<20}")
    print(f"  {'-'*60}")
    
    color_names = {
        (255, 0, 0): "파란색",
        (0, 255, 0): "초록색",
        (255, 255, 0): "시안",
        (0, 255, 255): "노란색",
        (0, 128, 255): "주황색",
        (0, 0, 255): "빨간색",
    }
    
    for distance in distances:
        b, g, r = get_color_from_distance(distance, max_distance)
        
        # 가장 가까운 색상 이름 찾기
        color_name = "혼합"
        for known_color, name in color_names.items():
            if (b, g, r) == known_color:
                color_name = name
                break
        
        print(f"  {distance:<10.1f} {b:<8} {g:<8} {r:<8} {color_name:<20}")


def main():
    """메인 함수"""
    print("\n" + "="*70)
    print("visualize_pcd.py 테스트 스크립트")
    print("="*70)
    
    # 테스트 목록
    tests = [
        ("VADAS 카메라 모델", test_vadas_camera_model),
        ("보정 데이터베이스", test_calibration_db),
        ("PCD 파일 로딩", test_pcd_loading),
        ("PCD 투영", test_projection),
        ("색상 매핑", test_color_mapping),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            if test_name in ["PCD 파일 로딩", "PCD 투영"]:
                result = test_func()
                results.append((test_name, result))
            else:
                test_func()
                results.append((test_name, True))
        except Exception as e:
            print(f"❌ {test_name} 실패: {e}")
            results.append((test_name, False))
    
    # 요약
    print("\n" + "="*70)
    print("테스트 요약")
    print("="*70)
    
    passed = sum(1 for _, result in results if result is True)
    failed = sum(1 for _, result in results if result is False)
    skipped = sum(1 for _, result in results if result is None)
    
    for test_name, result in results:
        status = "✓ 통과" if result is True else ("⚠ 스킵" if result is None else "❌ 실패")
        print(f"{status}: {test_name}")
    
    print(f"\n총: {passed}개 통과, {failed}개 실패, {skipped}개 스킵")
    
    if failed == 0 and passed > 0:
        print("\n✓ 모든 테스트 통과!")
    elif failed > 0:
        print("\n❌ 일부 테스트 실패. 위 오류를 확인하세요.")
    
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
