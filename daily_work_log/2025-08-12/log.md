# 2025년 8월 12일 작업 로그

## `main_pipeline.py` 및 `visualize_depth_comparison.py` 수정 내역

### 1. `NameError: name \'CalibrationDB\' is not defined` 해결 (해결)
- `main_pipeline.py`에서 `CalibrationDB`, `load_image`, `LidarCameraProjector`, `DEFAULT_CALIB`, `DEFAULT_LIDAR_TO_WORLD_v2` import 문을 파일 상단으로 이동하여 정의되기 전에 사용되는 문제를 해결했습니다.

### 2. `TypeError: show_interactive_projected_pcd_viewer() got an unexpected keyword argument \'depth_filtered_pcd\'` 해결 (해결)
- `visualize_depth_comparison.py` 파일의 `show_interactive_projected_pcd_viewer` 함수 정의에 `depth_filtered_pcd: Optional[o3d.geometry.PointCloud] = None` 인자를 추가했습니다.
- `_draw_projected_points` 함수에서 `color_override`가 "purple"일 경우 RGB 값 `(128, 0, 128)`을 사용하도록 로직을 추가했습니다.

### 3. `UnboundLocalError: local variable \'local_pan_x\' referenced before assignment` 해결 (해결)
- `visualize_depth_comparison.py` 파일의 `update_local_display_image` 함수에서 `local_pan_x`와 `local_pan_y`를 `nonlocal`로 선언하도록 수정했습니다.

### 4. `UnboundLocalError: local variable \'local_original_projected_image_cv\' referenced before assignment` 해결 (해결)
- `visualize_depth_comparison.py` 파일의 `update_local_display_image` 함수에서 `local_original_projected_image_cv`를 `nonlocal`로 선언하도록 수정했습니다.

### 5. 5미터 이내 포인트 시각화 및 표시 모드 전환 기능 추가 (해결)
- `visualize_depth_comparison.py` 파일에서 `depth_filtered_pcd`를 그릴 때 `color_override`를 "purple"로 변경했습니다.
- `show_interactive_projected_pcd_viewer` 함수에 `local_display_mode` 상태 변수를 추가하여 "ALL" (모든 PCD)과 "DEPTH_FILTERED" (깊이 필터링된 PCD만) 모드를 전환할 수 있도록 했습니다.
- \'p\' 키 입력을 감지하여 `local_display_mode`를 토글하고 화면을 갱신하는 로직을 추가했습니다.

### 6. 마우스 호버 시 좌표 정보 변경 (해결)
- `visualize_depth_comparison.py` 파일의 `mouse_callback_local` 함수를 수정하여 마우스 호버 시 표시되는 XYZ 좌표가 월드 좌표 대신 카메라 좌표를 나타내도록 변경했습니다. 레이블도 "Cam XYZ"로 변경했습니다.

### 7. 패닝 및 줌 속도 개선 (해결)
- `visualize_depth_comparison.py` 파일의 `show_interactive_projected_pcd_viewer` 함수에서 각 표시 모드("ALL", "DEPTH_FILTERED")에 대한 이미지를 미리 렌더링하여 저장하도록 수정했습니다.
- `update_local_display_image` 함수에서 포인트 그리기 로직을 제거하고, 미리 렌더링된 이미지를 사용하여 줌/패닝을 적용하도록 변경했습니다.

## 패키지 설치 및 환경 설정 (해결)

### 1. 필수 Python 패키지 설치 시도 (해결)
- `pip install "torch>=2.1" "transformers>=4.53.3" accelerate opencv-python pillow numpy` 명령어를 통해 패키지 설치를 시도했습니다.
- `torch` 및 `accelerate` 설치 중 `OSError: [Errno 2] No such file or directory` 오류가 발생하여 설치에 실패했습니다.

### 2. `torch` 및 GPU 연결 상태 확인 (해결)
- `python -c "import torch; print(f'Torch installed: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda}')"` 명령어를 통해 `torch` 설치 및 CUDA 연결 상태를 확인했습니다.
- 결과: `Torch installed: 2.8.0+cpu`, `CUDA available: False`, `CUDA version: None`. CPU 버전의 `torch`가 설치되어 있으며 GPU가 연결되지 않았음을 확인했습니다.

### 3. 시스템 CUDA 버전 확인 (해결)
- `nvcc --version` 명령어를 통해 시스템에 설치된 CUDA 버전을 확인했습니다.
- 결과: `Cuda compilation tools, release 11.1, V11.1.74`. 시스템 CUDA 버전이 11.1임을 확인했습니다.

### 4. CUDA 11.1용 `torch` 2.1 설치 시도 및 실패 (해결)
- `pip install torch==2.1.0+cu111 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu111 transformers>=4.53.3 opencv-python pillow numpy` 명령어를 통해 CUDA 11.1에 맞는 `torch` 2.1 설치를 시도했습니다.
- 결과: `ERROR: Could not find a version that satisfies the requirement torch==2.1.0+cu111`. `torch` 2.1 이상은 CUDA 11.7 또는 11.8 이상을 요구하므로 설치에 실패했습니다.

## 오늘 작업 내용 (2025-08-12)

### 1. `fisheye_depth_anything_v2.py` CUDA 사용 여부 디버깅
- `fisheye_depth_anything_v2.py` 스크립트의 `run_depth_anything_v2` 함수에 현재 사용 중인 디바이스(`cuda` 또는 `cpu`)를 출력하는 `print(f"[INFO] Using device: {device}")` 문을 추가했습니다.
- VS Code `launch.json` 파일에 `fisheye_depth_anything_v2.py` 실행을 위한 새로운 디버그 설정("Run fisheye_depth_anything_v2.py (CUDA)")을 추가했습니다. 이 설정은 `--image`, `--out`, `--device cuda`, `--fp16` 인자를 포함합니다.
- VS Code에서 `launch.json`에 정의된 인자를 사용하여 스크립트를 실행하는 방법을 안내했습니다.

### 2. `transformers` 라이브러리 경고 메시지 제거
- `fisheye_depth_anything_v2.py` 스크립트의 `run_depth_anything_v2` 함수 내 `AutoImageProcessor.from_pretrained()` 호출에 `use_fast=False` 인자를 명시적으로 추가하여 "Using a slow image processor..." 경고 메시지가 더 이상 출력되지 않도록 수정했습니다.

### 3. CUDA 환경 및 패키지 설치 완료 (해결)
- CUDA Toolkit 업데이트 및 최신 NVIDIA GPU 드라이버 설치를 완료했습니다.
- `torch` 2.1 이상 버전을 포함한 모든 필수 Python 패키지(`torch`, `transformers`, `accelerate`, `opencv-python`, `pillow`, `numpy`)를 성공적으로 재설치했습니다. 이제 `fisheye_depth_anything_v2.py` 스크립트가 CUDA를 사용하여 실행될 것입니다.

### 4. 깊이 맵 스케일 비교 및 시각화 계획

**목표:**
`fisheye_depth_anything_v2.py`에서 생성된 Depth Anything V2의 `raw.png` 깊이 결과와 `C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\ncdb-cls-sample\synced_data\depth_maps`에 있는 기존 깊이 맵(`0000000931` 파일)의 스케일을 비교하고 시각화한다.

**비교 대상 파일:**
1.  **Depth Anything V2 결과:** `C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\output\single_image_test.raw.png`
2.  **기존 깊이 맵:** `C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\ncdb-cls-sample\synced_data\depth_maps\0000000931.npy` (확장자는 확인 필요)

**분석 및 시각화 방법:**
*   두 깊이 맵을 로드한다.
*   각 맵의 통계량 (최소, 최대, 평균, 표준 편차)을 계산하여 비교한다.
*   `matplotlib`을 사용하여 두 맵을 나란히 표시하고, 컬러바를 통해 스케일을 시각적으로 비교한다.
*   두 맵의 픽셀 값 분포를 비교하기 위해 히스토그램을 그린다.
*   필요시 두 맵의 차이 맵을 생성하여 시각화한다.

## ToDo

- **Depth Anything V2 깊이 맵 스케일 조정:** `fisheye_depth_anything_v2.py`에서 생성된 상대 깊이 맵의 스케일을 `ncdb-cls-sample` 디렉토리의 PCD 데이터(절대 깊이 정보)를 이용하여 실제 스케일에 맞게 조정하는 기능을 구현해야 합니다.

## 오늘 작업 내용 (2025-08-12) - `compare_depth_scales.py` 수정

### 1. Depth Anything V2 출력값 스케일 조정
- `compare_depth_scales.py`에서 Depth Anything V2의 원본 출력값(`raw.npy`)을 불러온 후, 시차(Disparity)와 유사한 특성을 가지므로 값을 뒤집어 깊이(Depth)처럼 보이도록 변환했습니다 (`np.max(output) - output`).

### 2. 바이너리 마스크 적용 로직 개선
- 마스크 적용 시, 유효 영역 판단 기준을 `> 128`에서 `> 0`으로 변경하여 마스크의 흰색 부분이 어떤 양수 값을 가져도 정상적으로 인식되도록 수정했습니다.
- 마스크와 깊이 맵의 크기가 다를 경우 오류를 발생시키고, 크기가 같을 때만 위치를 기준으로 마스크를 적용하도록 수정했습니다 (`np.where` 사용).
- `compare_depth_scales.py` 파일의 마스크 적용 관련 코드를 위와 같이 수정했습니다.
