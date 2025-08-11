# 2025년 8월 11일 작업 로그

## 주요 작업 내용

- **X값 기준 LiDAR 라인 포인트 추출 및 시각화 개선:**
  - `main_pipeline.py`에 `find_lidar_line_by_x` 함수를 추가하여, 주어진 X 좌표와 허용 오차 범위 내에 있는 LiDAR PCD 포인트들을 추출하는 기능을 구현했습니다.
  - 추출된 라인 포인트와 원본 PCD를 `output` 디렉토리에 각각 PCD 및 3D 비교 시각화 이미지로 저장하도록 했습니다.
  - `visualize_depth_comparison.py`에 `show_interactive_projected_pcd_viewer` 함수를 추가하여, 원본 PCD와 라인 포인트를 이미지에 투영하고 마우스로 줌/패닝이 가능한 인터랙티브 뷰어를 구현했습니다.
  - `ref/camera_lidar_projector.py`의 `_get_color_from_distance` 함수를 업데이트하여, 투영된 포인트의 색상이 거리에 따라 동적으로 변하도록 컬러맵을 적용했습니다.

## 최종 결과물

- `output` 디렉토리에 `output_line_pcd.pcd` 파일이 생성되어 X값 기준으로 추출된 LiDAR 라인 포인트를 포함합니다.
- `output` 디렉토리에 `lidar_line_comparison_3d.png` 파일이 생성되어 원본 PCD와 추출된 LiDAR 라인 포인트를 3D로 비교 시각화한 이미지를 포함합니다.
- `output` 디렉토리에 `lidar_line_projected_comparison.png` 파일이 생성되어 원본 PCD와 추출된 LiDAR 라인 포인트의 이미지 투영을 비교 시각화한 이미지를 포함합니다.
- 인터랙티브 뷰어를 통해 원본 PCD와 라인 포인트의 투영을 실시간으로 확인하고 조작할 수 있습니다.