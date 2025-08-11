# 2025년 8월 11일 작업 로그

## 주요 작업 내용

- **X값 기준 LiDAR 라인 포인트 추출 기능 구현:**
  - `main_pipeline.py`에 `find_lidar_line_by_x` 함수를 추가하여, 주어진 X 좌표와 허용 오차 범위 내에 있는 LiDAR PCD 포인트들을 추출하는 기능을 구현했습니다.
  - 이 함수는 LiDAR 스캔 라인과 유사한 1채널 포인트를 식별하는 데 사용될 수 있습니다.

- **출력 폴더 관리:**
  - 모든 출력 파일이 `output`이라는 별도의 디렉토리에 저장되도록 `main_pipeline.py`를 수정했습니다.

- **원본-추출 라인 PCD 비교 시각화 기능 추가:**
  - `visualize_depth_comparison.py`에 `visualize_pcd_comparison` 함수를 재정의하여 원본 PCD와 추출된 LiDAR 라인 PCD를 다른 색상(원본: 회색, 라인: 빨간색)으로 시각화하고 PNG 이미지로 저장하는 기능을 구현했습니다.

- **기능 테스트 및 확인:**
  - `main_pipeline.py`의 메인 실행 블록에서 `find_lidar_line_by_x` 함수를 호출하여 특정 PCD 파일(`0000001996.pcd`)에서 X값을 기준으로 라인 포인트를 성공적으로 추출하고, `visualize_pcd_comparison` 함수를 통해 원본 PCD와 추출된 라인 PCD를 비교 시각화하는 것을 확인했습니다.

## 최종 결과물

- `output` 디렉토리에 `output_line_pcd.pcd` 파일이 생성되어 X값 기준으로 추출된 LiDAR 라인 포인트를 포함합니다.
- `output` 디렉토리에 `lidar_line_comparison.png` 파일이 생성되어 원본 PCD와 추출된 LiDAR 라인 포인트를 비교 시각화한 이미지를 포함합니다.