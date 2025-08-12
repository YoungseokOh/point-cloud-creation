# 2025년 8월 12일 작업 로그

## `main_pipeline.py` 및 `visualize_depth_comparison.py` 수정 내역

### 1. `NameError: name 'CalibrationDB' is not defined` 해결 (해결)
- `main_pipeline.py`에서 `CalibrationDB`, `load_image`, `LidarCameraProjector`, `DEFAULT_CALIB`, `DEFAULT_LIDAR_TO_WORLD_v2` import 문을 파일 상단으로 이동하여 정의되기 전에 사용되는 문제를 해결했습니다.

### 2. `TypeError: show_interactive_projected_pcd_viewer() got an unexpected keyword argument 'depth_filtered_pcd'` 해결 (해결)
- `visualize_depth_comparison.py` 파일의 `show_interactive_projected_pcd_viewer` 함수 정의에 `depth_filtered_pcd: Optional[o3d.geometry.PointCloud] = None` 인자를 추가했습니다.
- `_draw_projected_points` 함수에서 `color_override`가 "purple"일 경우 RGB 값 `(128, 0, 128)`을 사용하도록 로직을 추가했습니다.

### 3. `UnboundLocalError: local variable 'local_pan_x' referenced before assignment` 해결 (해결)
- `visualize_depth_comparison.py` 파일의 `update_local_display_image` 함수에서 `local_pan_x`와 `local_pan_y`를 `nonlocal`로 선언하도록 수정했습니다.

### 4. `UnboundLocalError: local variable 'local_original_projected_image_cv' referenced before assignment` 해결 (해결)
- `visualize_depth_comparison.py` 파일의 `update_local_display_image` 함수에서 `local_original_projected_image_cv`를 `nonlocal`로 선언하도록 수정했습니다.

### 5. 5미터 이내 포인트 시각화 및 표시 모드 전환 기능 추가 (해결)
- `visualize_depth_comparison.py` 파일에서 `depth_filtered_pcd`를 그릴 때 `color_override`를 "purple"로 변경했습니다.
- `show_interactive_projected_pcd_viewer` 함수에 `local_display_mode` 상태 변수를 추가하여 "ALL" (모든 PCD)과 "DEPTH_FILTERED" (깊이 필터링된 PCD만) 모드를 전환할 수 있도록 했습니다.
- 'p' 키 입력을 감지하여 `local_display_mode`를 토글하고 화면을 갱신하는 로직을 추가했습니다.

### 6. 마우스 호버 시 좌표 정보 변경 (해결)
- `visualize_depth_comparison.py` 파일의 `mouse_callback_local` 함수를 수정하여 마우스 호버 시 표시되는 XYZ 좌표가 월드 좌표 대신 카메라 좌표를 나타내도록 변경했습니다. 레이블도 "Cam XYZ"로 변경했습니다.

### 7. 패닝 및 줌 속도 개선 (해결)
- `visualize_depth_comparison.py` 파일의 `show_interactive_projected_pcd_viewer` 함수에서 각 표시 모드("ALL", "DEPTH_FILTERED")에 대한 이미지를 미리 렌더링하여 저장하도록 수정했습니다.
- `update_local_display_image` 함수에서 포인트 그리기 로직을 제거하고, 미리 렌더링된 이미지를 사용하여 줌/패닝을 적용하도록 변경했습니다.

## 패키지 설치 및 환경 설정

### 1. 필수 Python 패키지 설치 시도
- `pip install "torch>=2.1" "transformers>=4.53.3" accelerate opencv-python pillow numpy` 명령어를 통해 패키지 설치를 시도했습니다.
- `torch` 및 `accelerate` 설치 중 `OSError: [Errno 2] No such file or directory` 오류가 발생하여 설치에 실패했습니다.

### 2. `torch` 및 GPU 연결 상태 확인
- `python -c "import torch; print(f'Torch installed: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda}')"` 명령어를 통해 `torch` 설치 및 CUDA 연결 상태를 확인했습니다.
- 결과: `Torch installed: 2.8.0+cpu`, `CUDA available: False`, `CUDA version: None`. CPU 버전의 `torch`가 설치되어 있으며 GPU가 연결되지 않았음을 확인했습니다.

### 3. 시스템 CUDA 버전 확인
- `nvcc --version` 명령어를 통해 시스템에 설치된 CUDA 버전을 확인했습니다.
- 결과: `Cuda compilation tools, release 11.1, V11.1.74`. 시스템 CUDA 버전이 11.1임을 확인했습니다.

### 4. CUDA 11.1용 `torch` 2.1 설치 시도 및 실패
- `pip install torch==2.1.0+cu111 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu111 transformers>=4.53.3 opencv-python pillow numpy` 명령어를 통해 CUDA 11.1에 맞는 `torch` 2.1 설치를 시도했습니다.
- 결과: `ERROR: Could not find a version that satisfies the requirement torch==2.1.0+cu111`. `torch` 2.1 이상은 CUDA 11.7 또는 11.8 이상을 요구하므로 설치에 실패했습니다.

## ToDo

- **CUDA Toolkit 업데이트:** 현재 시스템에 설치된 CUDA Toolkit 11.1을 제거하고, NVIDIA 개발자 웹사이트에서 CUDA Toolkit 11.8을 다운로드하여 설치해야 합니다. (사용자 수동 작업 필요)
- **최신 NVIDIA GPU 드라이버 설치:** CUDA Toolkit 업데이트 전 또는 후에 최신 GPU 드라이버를 설치해야 합니다. (사용자 수동 작업 필요)
- **패키지 재설치:** CUDA Toolkit 업데이트 및 시스템 재부팅 완료 후, `torch` 2.1 이상 버전을 포함한 모든 필수 Python 패키지(`torch`, `transformers`, `accelerate`, `opencv-python`, `pillow`, `numpy`)를 다시 설치해야 합니다.