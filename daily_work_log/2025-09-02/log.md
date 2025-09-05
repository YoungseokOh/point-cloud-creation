### 2025년 9월 2일 화요일 작업 로그

#### `find_road_by_geometry.py` 수정 계획

**목표**: `find_road_by_geometry.py` 스크립트가 단일 PCD 파일 로드뿐만 아니라, 지정된 폴더 내의 모든 PCD 파일을 순회하며 로드하고 시각화할 수 있도록 기능을 확장한다. 키보드 입력으로 다음/이전 PCD 파일로 전환 가능하게 한다. 기존 알고리즘 및 시각화 로직은 유지한다.

**상세 계획**:

1.  **PCD 파일 로드 방식 변경**:
    *   `argparse` 모듈을 사용하여 스크립트 실행 시 단일 파일 경로 또는 폴더 경로를 명령줄 인자로 받도록 수정한다.
    *   입력 경로가 폴더인 경우, 해당 폴더 내의 모든 `.pcd` 파일을 찾아 리스트로 저장한다. 파일인 경우, 해당 파일만 리스트에 추가한다.

2.  **시각화 루프 구현**:
    *   `open3d.visualization.VisualizerWithKeyCallback`를 사용하여 키보드 콜백 함수를 등록한다.
    *   'n' 키 (next)를 누르면 다음 PCD 파일로, 'p' 키 (previous)를 누르면 이전 PCD 파일로 전환하도록 콜백 함수를 구현한다.
    *   현재 로드된 PCD 파일의 인덱스를 추적하고, 파일 리스트의 처음과 끝에 도달했을 때 인덱스 범위를 벗어나지 않도록 처리한다.
    *   각 파일이 로드될 때마다 `find_closest_line_by_euclidean_distance` 함수를 호출하여 분석을 수행하고, 시각화 객체(PCD, 원점 구)를 `VisualizerWithKeyCallback` 인스턴스에 추가하거나 업데이트한다.

3.  **기존 로직 유지**:
    *   `find_closest_line_by_euclidean_distance` 함수 내부의 핵심 알고리즘 (Z-필터링, XY 거리 필터링, X축 양수 제외, 3D 유클리드 거리 기반 최접점 찾기, 결과 PCD 저장)은 변경하지 않는다.
    *   시각화 방식만 `o3d.visualization.draw_geometries` 대신 `VisualizerWithKeyCallback` 인스턴스의 `add_geometry`, `update_geometry`, `remove_geometry` 등을 사용하여 동적으로 업데이트하도록 변경한다.

4.  **오류 처리 및 사용자 안내**:
    *   파일 또는 폴더가 존재하지 않거나, PCD 파일이 비어있는 경우 등 기존의 오류 처리 로직을 유지한다.
    *   사용자에게 키보드 조작 방법 ('n' for next, 'p' for previous, 'q' or 'esc' to quit)을 안내하는 메시지를 추가하여 사용 편의성을 높인다.
