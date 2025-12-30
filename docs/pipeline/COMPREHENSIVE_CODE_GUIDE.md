# integrated_pcd_depth_pipeline_newest.py - 완전 설명 및 사용 가이드

## 📌 문서 개요
이 문서는 `integrated_pcd_depth_pipeline_newest.py`의 **완전한 동작 원리**와 **실제 사용법**을 설명합니다.

---

## 🎯 1단계: 파이프라인 전체 흐름도

```
입력 데이터
    ↓
[1] 보정 데이터 로드 (calibration.json)
    - VADAS Fisheye 카메라 intrinsic 파라미터
    - 해상도별 변환 매트릭스
    ↓
[2] 각 PCD 파일 처리 (1001개 파일)
    ├─ [2-1] 원본 3D 포인트 로드 (binary PCD)
    │
    ├─ [2-2] 합성 포인트 생성
    │   - 지면에 동심원 패턴 생성
    │   - 각 동심원마다 100개 포인트
    │
    ├─ [2-3] 깊이맵 생성 (1920×1536 해상도)
    │   ├─ 원본만 투영 → depth_orig
    │   ├─ 합성만 투영 → depth_synth
    │   └─ 원본+합성 투영 → depth_merge
    │
    ├─ [2-4] 크기 조정 해상도별 처리 (640×512, 640×384)
    │   ├─ 카메라 intrinsic 스케일 적용
    │   │   (ux, uy만 스케일, div는 원본 유지)
    │   │
    │   ├─ 깊이맵 생성
    │   │   - KITTI 포맷: uint16 PNG = depth_meters * 256
    │   │
    │   └─ 시각화 생성
    │       ├─ RGB + Depth 산점도 (viz_results)
    │       ├─ Depth colormap (colormap)
    │       └─ 차이 깊이맵 (diff_results)
    │
    └─ [2-5] 출력 파일 저장
        └─ 다중 해상도 폴더에 저장

출력 디렉토리 구조
    ↓
depth_maps_newest/        (1920×1536 - 원본 해상도)
├─ newest_depth_maps/       (병합된 깊이맵)
├─ newest_synthetic_depth_maps/ (합성 깊이맵만)
├─ newest_viz_results/      (RGB+Depth 시각화)
├─ newest_colormap/         (Depth colormap)
├─ newest_pcd/             (PCD 파일)
└─ diff_results/           (차이 깊이맵)

640x512_newest/           (640×512 해상도 - 균일 스케일)
├─ newest_depth_maps/
├─ newest_synthetic_depth_maps/
├─ newest_viz_results/
├─ newest_colormap/
├─ newest_pcd/
└─ diff_results/

640x384_newest/           (640×384 해상도 - 비균일 스케일)
├─ newest_depth_maps/
├─ newest_synthetic_depth_maps/
├─ newest_viz_results/
├─ newest_colormap/
├─ newest_pcd/
└─ diff_results/
```

---

## 🔧 2단계: 핵심 개념 설명

### 2-1. VADAS Fisheye 카메라 모델

**Intrinsic 파라미터 (11개)**:
```python
intrinsic = [k[0], k[1], k[2], k[3], k[4], k[5], k[6],  # 다항식 계수 (polynomial)
             s,                                         # 크기 파라미터 (size)
             div,                                       # 왜곡 계수 (distortion)
             ux, uy]                                    # 주점 (principal point)
```

**중요한 규칙**:
- `k[0:7]`: 다항식 계수 - **절대 스케일링 금지** ❌
- `s`: 크기 파라미터 - **절대 스케일링 금지** ❌
- `div`: 왜곡 계수 - **절대 스케일링 금지** ❌
- `ux, uy`: 주점 - **반드시 스케일링 해야 함** ✅
- 최종 스케일: `scale_x, scale_y` - 투영된 픽셀 좌표에 적용 ✅

### 2-2. 스케일 적용 방식 (가로/세로 비율 유지)

**640×512 (균일 스케일)**:
```
원본: 1920×1536
스케일: 640/1920 = 512/1536 = 0.333 (동일)
→ scale_x = 0.333, scale_y = 0.333
```

**640×384 (비균일 스케일)**:
```
원본: 1920×1536
스케일: 640/1920 = 0.333, 384/1536 = 0.250 (다름)
→ scale_x = 0.333, scale_y = 0.250
```

**투영 공식** (개선된 방식 - Nov 14 수정):
```python
# 1단계: 다항식으로 거리 계산
rd = k[0] + k[1]*rho + k[2]*rho^2 + k[3]*rho^3 + k[4]*rho^4 + k[5]*rho^5 + k[6]*rho^6

# 2단계: 각도 구성 요소 계산
cosPhi = cx / sqrt(cx^2 + cy^2)
sinPhi = cy / sqrt(cx^2 + cy^2)

# 3단계: 스케일 적용 (div는 유지, 좌표에만 스케일 적용)
u = rd * cosPhi * scale_x + ux + img_w_half
v = rd * sinPhi * scale_y + uy + img_h_half
```

### 2-3. 깊이맵 저장 형식 (KITTI Convention)

```python
# 저장: depth_float → uint16
depth_uint16 = uint16(depth_meters * 256)
# PNG로 저장 (손실 없음)

# 읽기: uint16 → depth_float
depth_meters = uint16_value / 256.0
```

예시:
```
5.0 meters → 5.0 * 256 = 1280 → PNG에 저장
PNG에서 1280 읽음 → 1280 / 256.0 = 5.0 meters 복원
```

### 2-4. 합성 포인트 생성 (C-Circle Pattern)

```python
# 지면 (z=0)에 동심원 패턴 생성
# 반경: 2m, 5m, 8m, 10m, ... (2m 간격)

for each_radius in [2, 5, 8, 10, 12, ...]:
    for angle in range(0, 360, step=3.6):  # 100개 포인트
        x = radius * cos(angle)
        y = radius * sin(angle)
        z = 0  # 지면
        point = (x, y, z, intensity=10, t=0, ring=0)
```

**목적**:
- 깊이맵 품질 검증
- 가시 범위(FoV) 내 포인트 분포 확인
- 원본 데이터 부족한 영역 커버

---

## 💻 3단계: 주요 함수 설명

### 3-1. `VADASFisheyeCameraModel` 클래스

**초기화**:
```python
camera = VADASFisheyeCameraModel(
    intrinsic=[k0, k1, k2, k3, k4, k5, k6, s, div, ux, uy],
    image_size=(1920, 1536),
    camera_matrix=[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
)
```

**주요 메서드**:

```python
# 1. 스케일 적용 (특정 해상도로 변환할 때)
camera.scale_intrinsics(target_image_size=(640, 512))
# → ux, uy만 스케일링
# → scale_x, scale_y 저장 (투영 시 사용)

# 2. 3D 점 투영 (카메라 좌표계 → 이미지 좌표)
u, v, valid = camera.project_point(
    cx,  # 카메라 X (오른쪽)
    cy,  # 카메라 Y (아래)
    cz   # 카메라 Z (앞, 깊이)
)
# 반환: (u, v) = 이미지 좌표, valid = 이미지 범위 내 여부
```

### 3-2. `LidarCameraProjector` 클래스

**깊이맵 생성** (핵심 함수):
```python
depth_map, provenance_map = projector.project_cloud_to_depth_map_with_labels(
    camera_name="camera_0",           # 카메라 이름
    points=np.array([[x1,y1,z1], ...]),  # 3D 포인트 배열
    labels=np.array([0, 1, 1, ...]),  # 각 포인트의 라벨
                                       # 0=원본, 1=합성
    image_size=(1920, 1536)           # 목표 해상도
)
# 반환:
#   - depth_map: float32, 깊이값 (미터)
#   - provenance_map: int8, 포인트 출처
#       0 = 원본만
#       1 = 합성 포인트
```

**동작 원리**:
```
1. 주어진 image_size로 카메라 스케일 설정
   → scale_intrinsics(image_size) 호출
   → ux, uy가 새로운 해상도에 맞게 조정됨

2. 모든 3D 포인트를 이미지 좌표로 투영
   → project_point() 호출
   → (u, v) 이미지 좌표 계산

3. 각 픽셀에 가장 가까운 포인트만 저장
   → 중첩되는 포인트는 깊이값으로 정렬

4. 깊이맵과 provenance 맵 반환
```

### 3-3. 깊이맵 저장

```python
def save_depth_map(output_path, depth_map):
    """
    깊이맵을 uint16 PNG로 저장 (KITTI 포맷)
    
    Args:
        output_path: 저장 경로
        depth_map: float32 배열 (깊이값, 미터)
    """
    depth_uint16 = np.clip(depth_map * 256, 0, 65535).astype(np.uint16)
    cv2.imwrite(str(output_path), depth_uint16)
```

### 3-4. RGB + Depth 시각화

```python
def create_rgb_with_depth_scatter(rgb_image, depth_map, point_size=2, max_depth=15.0):
    """
    RGB 이미지 위에 깊이값을 색상으로 표시
    
    Args:
        rgb_image: BGR 이미지 (uint8)
        depth_map: 깊이맵 (float32, 미터)
        point_size: 포인트 크기 (픽셀)
        max_depth: 최대 깊이값 (이상 = 붉은색)
    
    Returns:
        RGB+Depth 오버레이된 이미지
    
    색상:
        - 검정색: 깊이값 없음 (0)
        - 초록색: 2m ~ 5m (가까움)
        - 노란색: 5m ~ 10m (중간)
        - 빨강색: 10m+ (먼것)
    """
```

---

## 📊 4단계: 출력 파일 설명

### 4-1. depth_maps_newest 폴더 (1920×1536)

| 파일 | 설명 | 포맷 |
|------|------|------|
| `0000000000.png` | 병합된 깊이맵 (원본+합성) | uint16 PNG (KITTI) |
| `0000000000_colored.png` | 깊이값을 색상으로 변환 | RGB PNG |
| `0000000000_depth_analysis.png` | RGB + Depth 오버레이 | RGB PNG |
| `0000000000_merged.png` | 깊이 차이 시각화 | RGB PNG |
| `0000000000_synth.png` | 합성 포인트 깊이 차이 | RGB PNG |
| `0000000000_orig.png` | 원본 포인트 깊이 차이 | RGB PNG |
| `0000000000.pcd` | 병합된 PCD 파일 | Binary PCD |

### 4-2. 640x512_newest, 640x384_newest 폴더

동일한 구조, 다른 해상도:
```
newest_depth_maps/         (실제 깊이 데이터)
newest_synthetic_depth_maps/ (합성만)
newest_viz_results/        (RGB+Depth 시각화)
newest_colormap/           (깊이 colormap)
newest_pcd/               (PCD 파일)
diff_results/             (깊이 차이)
```

### 4-3. 색상 범례 (Colormap)

```
Jet colormap (OpenCV):
    파란색   → 0m (너무 가까움)
    초록색   → ~5m (가까움)
    노란색   → ~10m (중간)
    빨강색   → 15m+ (먼것)
```

---

## 🚀 5단계: 명령어 사용 방법

### 5-1. 기본 사용법

```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir <데이터_경로> \
    --output_dir <출력_경로> \
    --calibration_path <보정_파일>
```

### 5-2. 전체 명령어 옵션

```bash
python integrated_pcd_depth_pipeline_newest.py \
    # [필수] 입력/출력
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/pipeline_results" \
    --calibration_path "./calibration.json" \
    
    # [선택] 크기 조정 해상도
    --resized_resolutions "640x512,640x384" \
    
    # [선택] 처리 설정
    --start_idx 0 \
    --end_idx 100 \
    --skip_existing \
    
    # [선택] 합성 포인트
    --max_radius 15.0 \
    --num_circles 8 \
    --points_per_circle 100 \
    
    # [선택] 시각화
    --max_depth 15.0 \
    --colormap_type jet \
    --point_size 2 \
    
    # [선택] 성능
    --num_workers 4 \
    --batch_size 10
```

### 5-3. 실제 사용 예제

#### 예제 1: 기본 실행 (전체 데이터)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/full_pipeline" \
    --calibration_path "./calibration.json"
```

#### 예제 2: 부분 처리 (처음 100개 파일만)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/test_100" \
    --calibration_path "./calibration.json" \
    --start_idx 0 \
    --end_idx 100
```

#### 예제 3: 빠른 테스트 (기존 파일 건너뛰고 10개만)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/quick_test" \
    --calibration_path "./calibration.json" \
    --end_idx 10 \
    --skip_existing
```

#### 예제 4: 높은 해상도만 (1920×1536)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/high_res_only" \
    --calibration_path "./calibration.json" \
    --resized_resolutions ""
```

#### 예제 5: 커스텀 합성 패턴
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/custom_synth" \
    --calibration_path "./calibration.json" \
    --max_radius 20.0 \
    --num_circles 10 \
    --points_per_circle 50
```

---

## 📋 6단계: 명령어 인자 상세 설명

### 입력/출력 인자
```
--input_dir <경로>
    설명: PCD 파일 및 RGB 이미지가 있는 입력 디렉토리
    예: "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data"
    필수: YES

--output_dir <경로>
    설명: 처리 결과를 저장할 출력 디렉토리
    예: "./output/pipeline_results"
    필수: YES
    생성: 디렉토리가 없으면 자동 생성

--calibration_path <경로>
    설명: 카메라 보정 정보 (JSON 파일)
    예: "./calibration.json"
    필수: YES
    형식: JSON with intrinsic, extrinsic, image_size
```

### 해상도 인자
```
--resized_resolutions <해상도_목록>
    설명: 생성할 크기 조정 해상도
    기본값: "640x512,640x384"
    예: "640x512,640x384,480x360"
    예: "" (크기 조정 안함, 원본만 처리)
    형식: "WxH,WxH,..." (쉼표로 구분)
```

### 처리 범위 인자
```
--start_idx <숫자>
    설명: 처리 시작 파일 인덱스 (0부터 시작)
    기본값: 0
    예: 0

--end_idx <숫자>
    설명: 처리 종료 파일 인덱스 (포함하지 않음)
    기본값: -1 (모든 파일)
    예: 100 (0~99 파일 처리)

--skip_existing
    설명: 기존 출력 파일이 있으면 건너뛰기
    기본값: False (재처리)
    사용법: 플래그만 사용 (--skip_existing)
```

### 합성 포인트 인자
```
--max_radius <값>
    설명: 동심원 패턴의 최대 반경 (미터)
    기본값: 15.0
    예: 20.0 (2m~20m 동심원)

--num_circles <숫자>
    설명: 동심원의 개수
    기본값: 8
    예: 10 (총 10개 동심원)

--points_per_circle <숫자>
    설명: 각 동심원 위의 포인트 개수
    기본값: 100
    예: 50 (각 원에 50개 포인트)
```

### 시각화 인자
```
--max_depth <값>
    설명: 깊이맵 시각화의 최대 깊이 (미터)
    기본값: 15.0
    색상: 이 값 이상은 빨간색
    예: 20.0

--colormap_type <타입>
    설명: Colormap 유형
    기본값: "jet"
    선택지: "jet", "viridis", "hot", "cool"

--point_size <숫자>
    설명: RGB+Depth 시각화의 포인트 크기 (픽셀)
    기본값: 2
    예: 4 (더 큰 점)
```

### 성능 인자
```
--num_workers <숫자>
    설명: 병렬 처리 워커 수
    기본값: 4
    예: 8 (더 많은 병렬 처리)

--batch_size <숫자>
    설명: 배치 처리 크기
    기본값: 10
    예: 20 (한 번에 더 많은 파일 처리)
```

---

## 🔍 7단계: 실행 결과 확인

### 7-1. 로그 출력 이해

```
[INFO] Loading calibration from ./calibration.json
[INFO] Initializing pipeline with:
  - Input: D:/data/...
  - Output: ./output/pipeline_results
  - Resized resolutions: [(640, 512), (640, 384)]
[INFO] Found 1001 PCD files

[PROCESS] File 0/1001: 0000000000.pcd
[LOAD] Loaded 50000 original points from PCD
[SYNTH] Generated 800 synthetic points (8 circles, 100 points each)
[MERGE] Created merged point cloud: 50800 points
[DEPTH] Generated depth_map for 1920x1536
[SCALE] Processing resolution 640x512...
[DEPTH] Generated depth_map for 640x512
[SAVE] RGB+Depth visualization saved: 0000000000_depth_analysis.png
[SAVE] Colormap saved: 0000000000_colored.png
[COMPLETE] File 0/1001 finished

...

[FINISH] Pipeline complete!
[STATS] Total time: 2h 34m
[STATS] Average time per file: 9.2s
```

### 7-2. 출력 파일 검증

**생성되어야 하는 파일**:
```
output/pipeline_results/
├─ depth_maps_newest/
│  ├─ newest_depth_maps/
│  │  ├─ 0000000000.png        ✓ 존재해야 함
│  │  ├─ 0000000001.png
│  │  └─ ...
│  ├─ newest_viz_results/
│  │  ├─ 0000000000_depth_analysis.png  ✓ 중요!
│  │  └─ ...
│  └─ newest_colormap/
│     ├─ 0000000000_colored.png
│     └─ ...
│
├─ 640x512_newest/
│  └─ (동일한 구조)
│
└─ 640x384_newest/
   └─ (동일한 구조)
```

**파일 크기 확인**:
```bash
# 깊이맵 (uint16 PNG) - 약 400KB~600KB
D:\> ls -la output/pipeline_results/depth_maps_newest/newest_depth_maps/*.png
-rw-r--r--  1 user  group  450000 Nov 25 10:30 0000000000.png
-rw-r--r--  1 user  group  445000 Nov 25 10:31 0000000001.png

# RGB+Depth 시각화 - 약 2MB~4MB
D:\> ls -la output/pipeline_results/depth_maps_newest/newest_viz_results/*.png
-rw-r--r--  1 user  group 2500000 Nov 25 10:30 0000000000_depth_analysis.png
```

---

## 🐛 8단계: 문제 해결 (Troubleshooting)

### 문제 1: "No such file or directory"
```
ERROR: FileNotFoundError: [Errno 2] No such file or directory: 'calibration.json'
```
**해결책**:
```bash
# 보정 파일 경로 확인
ls -la calibration.json

# 절대 경로로 실행
python integrated_pcd_depth_pipeline_newest.py \
    --calibration_path "C:/full/path/to/calibration.json"
```

### 문제 2: "Input directory is empty"
```
ERROR: Input directory 'D:/data/...' contains no PCD files
```
**해결책**:
```bash
# PCD 파일 확인
ls D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data/*.pcd | head -5

# RGB 이미지 확인
ls D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data/image_a6/*.jpg | head -5
```

### 문제 3: "Depth map all zeros"
```
출력 이미지가 검정색만 나오는 경우
```
**해결책**:
```python
# 1. 포인트가 이미지 범위 내에 있는지 확인
# 2. 카메라 intrinsic 파라미터 확인
# 3. 깊이값 범위 확인
print(f"Min depth: {depth_map.min()}, Max depth: {depth_map.max()}")
print(f"Non-zero pixels: {np.count_nonzero(depth_map)}")
```

### 문제 4: "Out of memory"
```
MemoryError: Unable to allocate X GiB
```
**해결책**:
```bash
# 배치 크기 줄이기
python integrated_pcd_depth_pipeline_newest.py \
    --batch_size 5 \
    --num_workers 2

# 또는 부분 처리
python integrated_pcd_depth_pipeline_newest.py \
    --start_idx 0 \
    --end_idx 100
```

### 문제 5: "Windows encoding error (cp949)"
```
UnicodeEncodeError: 'cp949' codec can't encode character
```
**해결책**: 코드 시작 부분에서 자동 처리됨
```python
# main() 함수 시작에 추가됨:
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')
```

---

## 📊 9단계: 성능 최적화 팁

### 9-1. 빠른 처리
```bash
# 병렬 처리 최대화
python integrated_pcd_depth_pipeline_newest.py \
    --num_workers 8 \
    --batch_size 20
```

### 9-2. 메모리 절약
```bash
# 순차 처리
python integrated_pcd_depth_pipeline_newest.py \
    --num_workers 1 \
    --batch_size 5
```

### 9-3. 빠른 테스트
```bash
# 처음 10개만 처리
python integrated_pcd_depth_pipeline_newest.py \
    --end_idx 10
```

### 9-4. 기존 파일 건너뛰기
```bash
# 이미 완료된 파일은 스킵
python integrated_pcd_depth_pipeline_newest.py \
    --skip_existing
```

---

## 🎓 10단계: 고급 사용법

### 10-1. 원본 해상도만 처리
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "..." \
    --calibration_path "..." \
    --resized_resolutions ""
```

### 10-2. 추가 해상도 지원
```bash
# 480×360 해상도도 추가 생성
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "..." \
    --calibration_path "..." \
    --resized_resolutions "640x512,640x384,480x360"
```

### 10-3. 커스텀 시각화 설정
```bash
# 더 큰 포인트, 더 먼 깊이까지 표시
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "..." \
    --calibration_path "..." \
    --point_size 4 \
    --max_depth 25.0
```

### 10-4. 커스텀 합성 포인트
```bash
# 20m까지, 10개 원, 각각 50개 포인트
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "..." \
    --calibration_path "..." \
    --max_radius 20.0 \
    --num_circles 10 \
    --points_per_circle 50
```

---

## ✅ 11단계: 체크리스트

파이프라인 실행 전에 확인하세요:

- [ ] `calibration.json` 파일 준비됨
- [ ] 입력 디렉토리에 PCD 파일이 1000+개 있음
- [ ] 입력 디렉토리에 `image_a6` 폴더에 JPG/PNG 파일이 있음
- [ ] 출력 디렉토리에 쓰기 권한이 있음
- [ ] 충분한 디스크 공간 있음 (약 500GB)
- [ ] Python 3.8+ 설치됨
- [ ] OpenCV, NumPy, Pillow 설치됨

**설치 확인**:
```bash
python -c "import cv2; import numpy; import PIL; print('All required packages installed')"
```

---

## 📝 12단계: 출력 해석 가이드

### 깊이맵 PNG 파일 읽기

```python
import cv2
import numpy as np

# 저장된 깊이맵 로드
depth_uint16 = cv2.imread('0000000000.png', cv2.IMREAD_UNCHANGED)

# uint16 → float32 (미터 단위)
depth_meters = depth_uint16.astype(np.float32) / 256.0

print(f"Min depth: {depth_meters.min():.2f}m")
print(f"Max depth: {depth_meters.max():.2f}m")
print(f"Mean depth: {depth_meters.mean():.2f}m")
print(f"Valid pixels: {np.count_nonzero(depth_meters)}")
```

### RGB+Depth 시각화 해석

```
검정색 배경: 포인트가 투영되지 않은 영역
초록/파란색: 2m~5m (가까운 객체)
노란색: 5m~10m (중간 거리)
빨강색: 10m+ (먼 거리)
```

### 합성 포인트의 역할

```
원본 깊이맵 vs 병합 깊이맵 비교:
- 원본만: 실제 센서 데이터 (산발적, 노이즈 있을 수 있음)
- 합성만: 이상적인 지면 패턴
- 병합: 실제 데이터 + 검증 패턴
```

---

## 📌 요약

이 파이프라인은:

1. **다중 해상도 깊이맵 생성**: 1920×1536, 640×512, 640×384
2. **Fisheye 카메라 투영**: 11-parameter VADAS 모델
3. **합성 포인트 검증**: C-circle 패턴으로 FoV 검증
4. **RGB+Depth 시각화**: 실제 센서와 합성 데이터의 직관적 비교
5. **KITTI 포맷**: uint16 PNG로 손실 없이 저장

**핵심은 스케일링**:
- `div` 파라미터는 절대 스케일링하면 안됨
- `ux, uy`만 스케일링하고 좌표에 적용
- 비균일 스케일(640×384)도 올바르게 처리 가능

