# integrated_pcd_depth_pipeline_newest.py - 전체 구조 설명

## 📋 목차
1. [파일 목적](#파일-목적)
2. [전체 실행 흐름](#전체-실행-흐름)
3. [핵심 구성요소](#핵심-구성요소)
4. [명령어 및 사용법](#명령어-및-사용법)
5. [입출력 경로](#입출력-경로)
6. [각 함수/클래스 상세](#각-함수클래스-상세)

---

## 파일 목적

**목표**: LiDAR 포인트 클라우드 (.pcd) → 깊이 맵 (depth map) 생성 + 시각화

이 파이프라인은:
- ✅ 원본 포인트 클라우드에서 **도로에 가장 가까운 포인트들**을 추출 (closest-line)
- ✅ 그 포인트들 주변에 **검은 고리(c-circles) 형태의 합성 포인트** 생성
- ✅ 원본 + 합성 포인트를 **카메라로 투영**하여 깊이 맵 생성
- ✅ 다양한 **해상도**로 출력 (1920×1536, 640×384 등)
- ✅ **시각화 이미지** 생성 (컬러맵, 분석 플롯)

---

## 전체 실행 흐름

```
┌─────────────────────────────────────────────────────────────┐
│  터미널에서 명령어 실행                                      │
│  python integrated_pcd_depth_pipeline_newest.py              │
│    --parent_folder <경로>                                    │
│    [--camera a6]                                             │
│    [--ground_z_min -0.95]                                    │
│    [--ground_z_max 0.5]                                      │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────┐
│ main() 함수                                                   │
│ ├─ 명령줄 인자 파싱                                          │
│ └─ run_integrated_pipeline() 호출                            │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────┐
│ run_integrated_pipeline()                                    │
│                                                               │
│ 1️⃣ 입력/출력 경로 설정                                       │
│    ├─ 입력: pcd/*.pcd 파일들                                 │
│    └─ 출력: newest_pcd/, newest_depth_maps/, 등             │
│                                                               │
│ 2️⃣ 모든 PCD 파일에 대해 반복 처리                            │
│    (tqdm 진행률 표시)                                        │
│                                                               │
│    각 PCD 파일마다:                                          │
│    ├─ Step 1: 도로 위 최가까운 점 추출                       │
│    ├─ Step 2: 합성 포인트(c-circles) 생성                   │
│    ├─ Step 3: 원본 + 합성 포인트 병합                        │
│    ├─ Step 4: 깊이 맵 생성                                   │
│    │           (1920×1536 해상도)                           │
│    ├─ Step 5: 640×384 해상도로도 생성 (선택)                │
│    └─ Step 6: 시각화/컬러맵/diff 이미지 저장                │
│                                                               │
│ 3️⃣ 최종 요약 출력                                            │
└──────────────────────────────────────────────────────────────┘
```

---

## 핵심 구성요소

### 1️⃣ **카메라 모델** (VADAS Fisheye Camera Model)

#### `VADASFisheyeCameraModel` 클래스

**역할**: 3D 포인트 (Xc, Yc, Zc) → 2D 픽셀 좌표 (u, v) 변환

**내부 파라미터**:
```python
- k: 다항식 계수 (polynomial coefficients) 7개
- s: 스케일 파라미터
- div: 정규화 파라미터 (radial distance normalization)
- ux, uy: 주점 오프셋 (principal point offset)
- image_size: 이미지 해상도 (width, height)
```

**투영 공식**:
```
1. 3D → 각도: theta = atan2(sqrt(Yc² + Zc²), Xc)
2. 각도 스케일: xd = theta * s
3. 다항식: rd = poly(xd) / div
4. 2D 좌표: u = rd * cos(phi) + ux + width/2
            v = rd * sin(phi) + uy + height/2
```

**주요 메서드**:
- `project_point(Xc, Yc, Zc)`: 단일 점 투영 → (u, v, valid_flag)
- `scale_intrinsics(scale_x, scale_y)`: 해상도 변경 시 ux, uy 스케일링

---

### 2️⃣ **보정 데이터베이스** (CalibrationDB)

**역할**: 카메라 캘리브레이션 정보 관리

```python
CalibrationDB
├─ sensors: {카메라이름 → SensorInfo}
│   └─ SensorInfo
│       ├─ name: "a6"
│       ├─ model: VADASFisheyeCameraModel
│       ├─ intrinsic: [k0...k6, s, div, ux, uy]
│       ├─ extrinsic: 4×4 변환 행렬 (LiDAR → Camera)
│       └─ image_size: (1920, 1536)
└─ lidar_to_world: 4×4 변환 행렬
```

**데이터 소스**: `DEFAULT_CALIB` (코드 상단에 정의된 딕셔너리)

---

### 3️⃣ **투영 엔진** (LidarCameraProjector)

**역할**: 포인트 클라우드 → 깊이 맵 변환

**주요 메서드**:

#### `project_cloud_to_depth_map(sensor_name, cloud_xyz, image_size)`
- 입력: 포인트 배열 (N×3), 해상도 (width, height)
- 출력: 깊이 맵 (height×width, float32)
- 처리: 
  1. 좌표계 변환 (LiDAR → Camera)
  2. 각 포인트 투영
  3. 오클루전 검사 (가장 가까운 점만 유지)

#### `project_cloud_to_depth_map_with_labels(sensor_name, cloud_xyz, labels, image_size)`
- 입력: 라벨 배열 (원본=0, 합성=1)
- 출력: 깊이 맵 + provenance (어느 데이터가 우선인지 추적)

---

### 4️⃣ **포인트 생성** (도로 분석)

#### `find_nearest_road_point_and_generate_synthetic_pcd()`

**역할**: 도로 위 최가까운 포인트 찾기

**단계**:
1. Z 범위 필터 (지면): `ground_z_min < z < ground_z_max`
2. XY 거리 필터: `min_xy_distance < xy_dist < xy_radius_threshold`
3. X 필터: `x <= 0` (양수 제외)
4. 방위각 각 bin별 최가까운 점 선택 (3D 유클리드 거리)

**출력**:
- `closest_line_points`: 도로 위 추출된 점들 (N×3)
- `original_points`: 전체 원본 포인트 (M×3)

---

#### `build_c_circles_points_from_cloud()`

**역할**: 합성 포인트(검은 고리) 생성

**단계**:
1. "보라색 점" 선택: |y| ≤ 0.01 && x ≤ 0 중 가장 가까운 점
2. 틀어진 기준축 구성: 보라색 점 → (0, 0, -0.771) 벡터
3. 반경들 구성: 이 벡터 길이 따라 여러 반경 생성
4. 각 반경마다 원을 360도로 분할한 점들 생성
5. x ≤ 0 영역만 유지

**출력**: 합성 포인트들 (K×3)

---

## 명령어 및 사용법

### 기본 명령어

```bash
python integrated_pcd_depth_pipeline_newest.py --parent_folder <경로>
```

### 예제 1: 기본 실행
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --parent_folder ncdb-cls-sample/synced_data
```

### 예제 2: 전체 옵션 지정
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --parent_folder ncdb-cls-sample/synced_data \
    --camera a6 \
    --ground_z_min -0.95 \
    --ground_z_max 0.5 \
    --min_xy_distance 1.0 \
    --xy_radius_threshold 10.0 \
    --diff_point_size 3 \
    --diff_point_iterations 1
```

### 주요 옵션 설명

| 옵션 | 기본값 | 설명 |
|------|------|------|
| `--parent_folder` | **필수** | 입력 폴더 (pcd 서브폴더 포함) |
| `--camera` | `"a6"` | 카메라 이름 |
| `--ground_z_min` | `-0.95` | 지면 Z 범위 최소값 |
| `--ground_z_max` | `0.5` | 지면 Z 범위 최대값 |
| `--min_xy_distance` | `1.0` | 최소 XY 거리 필터 |
| `--xy_radius_threshold` | `10.0` | 최대 XY 거리 필터 |
| `--diff_point_size` | `3` | Diff 이미지의 점 크기 |
| `--diff_point_iterations` | `1` | Diff 이미지의 팽창 반복 횟수 |

---

## 입출력 경로

### 입력 경로

```
parent_folder/
└── pcd/
    ├── 0000000931.pcd
    ├── 0000000932.pcd
    └── ...
```

> **경로 해석**: `--parent_folder`는 다음 중 하나:
> 1. `pcd/` 폴더를 포함하는 폴더
> 2. `pcd/` 폴더 자체
> 3. `.pcd` 파일들이 직접 있는 폴더

### 출력 경로 (1920×1536 해상도)

```
parent_folder/
├── newest_pcd/                    # 합성된 PCD 파일들
│   ├── 0000000931.pcd
│   └── ...
│
├── newest_depth_maps/             # 깊이 맵 (16-bit PNG)
│   ├── 0000000931.png
│   └── ...
│
├── newest_viz_results/            # 분석 플롯
│   ├── 0000000931_depth_analysis.png
│   └── ...
│
├── newest_colormap/               # 컬러맵 시각화
│   ├── 0000000931_colorized.png
│   └── ...
│
├── newest_synthetic_depth_maps/   # 합성 포인트만의 깊이 맵
│   ├── 0000000931.png
│   └── ...
│
└── diff_results/                  # Diff 이미지 (컬러)
    ├── 0000000931_merged.png
    ├── 0000000931_synth.png
    └── 0000000931_orig.png
```

### 출력 경로 (640×384 해상도, 선택사항)

```
parent_folder/
└── 640x384_newest/
    ├── newest_pcd/
    ├── newest_depth_maps/
    ├── newest_viz_results/
    ├── newest_colormap/
    ├── newest_synthetic_depth_maps/
    └── diff_results/
```

> **Note**: `640x384_newest/` 폴더가 이미 존재하면 건너뜀 (중복 처리 방지)

---

## 각 함수/클래스 상세

### `find_nearest_road_point_and_generate_synthetic_pcd()`

**위치**: 라인 36-210

**입력**:
```python
pcd_path: Path                    # PCD 파일 경로
ground_z_min: float = -3.0       # 지면 Z 최소값
ground_z_max: float = 0.0        # 지면 Z 최대값
min_xy_distance_from_origin: float = 2.0
xy_radius_threshold: float = 10.0
y_min, y_max: Optional[float]
num_radius_divisions: int = 20   # (호환성, 미사용)
points_per_circle: int = 200     # (호환성, 미사용)
keep_original_points: bool = True
exclude_outermost_circle: bool = True
```

**출력**:
```python
Tuple[np.ndarray, Optional[np.ndarray]]
# (closest_line_points, original_points_or_none)
```

**핵심 로직**:
```python
1. PCD 파일 로드 (binary 또는 ASCII)
2. 지면 Z 범위로 필터: ground_z_min < z < ground_z_max
3. XY 거리로 필터: min_xy_distance < ||xy|| < xy_radius_threshold
4. X ≤ 0 필터 적용 (양수 제외)
5. 360도를 N개 bin으로 나눔
6. 각 bin의 최가까운 점(3D 거리) 선택
7. 반환: (선택된 점들, 원본 전체 점들)
```

---

### `build_c_circles_points_from_cloud()`

**위치**: 라인 230-362

**입력**:
```python
all_points_np: np.ndarray         # 전체 포인트 (M×3)
y_zero_band: float = 0.01        # y≈0 대역폭
blue_depth_z: float = -0.771     # 기준점 Z
num_c_radii: int = NUM_C_RADII   # 반경 수
circle_segs: int = CIRCLE_SEGS   # 각 반경당 점 수 (360도 분할)
```

**출력**:
```python
np.ndarray  # 합성 포인트들 (K×3)
```

**생성 원리**:

```
Step 1: "보라색 점" 선택
  ├─ 조건: |y| ≤ 0.01 && x ≤ 0
  ├─ 선택 기준: 최소 3D 거리
  └─ purple = (x_p, y_p, z_p)

Step 2: 틀어진 좌표계 구성
  ├─ c_vec = purple - (0, 0, blue_depth_z)
  ├─ c_len = ||c_vec||
  └─ 회전축 u_elev, u_tan 계산

Step 3: 반경 분할
  ├─ radii = [r1, r2, ..., r_N]
  ├─ 각 r마다 원 생성 (circle_segs 등분)
  └─ 각 원: 중심 (0,0,blue_depth_z) + 반지름 r

Step 4: 필터링
  ├─ x ≤ 0 영역만 유지
  └─ 원본 점들과의 XY 거리로 추가 필터 (optional)

Result: 검은 고리 형태의 합성 포인트 집합
```

---

### `CalibrationDB` 클래스

**위치**: 라인 435-470

**역할**: 카메라 보정 정보 저장/관리

**메서드**:
```python
CalibrationDB(calib_dict, lidar_to_world=None)
  # calib_dict: {카메라이름: {model, intrinsic, extrinsic, image_size}}
  # lidar_to_world: 4×4 변환 행렬

.get(name: str) -> SensorInfo
  # 카메라 정보 조회
```

**내부 구조**:
```python
self.sensors = {
    "a6": SensorInfo(
        name="a6",
        model=VADASFisheyeCameraModel(...),
        intrinsic=[...],
        extrinsic=4x4_matrix,
        image_size=(1920, 1536)
    )
}
self.lidar_to_world = 4x4_identity_or_custom
```

---

### `LidarCameraProjector` 클래스

**위치**: 라인 471-587

**역할**: 포인트 클라우드 → 깊이 맵 변환

**메서드**:

#### `project_cloud_to_depth_map(sensor_name, cloud_xyz, image_size)`

```python
입력:
  sensor_name: str              # "a6" 등
  cloud_xyz: np.ndarray (N×3)   # LiDAR 좌표계의 포인트
  image_size: (width, height)   # 예: (1920, 1536) 또는 (640, 384)

출력:
  depth_map: np.ndarray (height×width, float32)
  # 각 픽셀의 깊이값 (미터, Xc 값)
```

**내부 처리**:
```python
1. 카메라 모델 준비
   └─ scale_intrinsics() 호출 (해상도가 다르면)

2. 좌표 변환
   └─ 3D_LiDAR → Camera: transform @ lidar_point

3. 포인트 필터링
   ├─ Xc > 0 (카메라 앞쪽만)
   └─ 특정 XY 범위 제외

4. 각 포인트 투영
   ├─ camera_model.project_point(Xc, Yc, Zc)
   └─ (u, v) 좌표 획득

5. 깊이 맵 기록 (오클루전 검사)
   └─ if depth_map[v, u] == 0 or depth_map[v, u] > Xc:
        depth_map[v, u] = Xc
```

#### `project_cloud_to_depth_map_with_labels(sensor_name, cloud_xyz, labels, image_size)`

```python
입력:
  labels: np.ndarray (N,)  # 0=원본, 1=합성

출력:
  depth_map: np.ndarray (height×width, float32)
  provenance: np.ndarray (height×width, int8)
             # 각 픽셀의 출처 추적 (-1: empty, 0: orig, 1: synth)
```

---

### `save_depth_map(path, depth_map)`

**위치**: 라인 622-638

**역할**: 깊이 맵을 16-bit PNG로 저장

```python
입력:
  path: Path              # 저장 경로
  depth_map: np.ndarray   # float32 깊이값 (미터)

처리:
  1. depth * 256 → uint16 변환
  2. 16-bit PNG로 저장
```

**예**:
```
깊이값 1.5m → 1.5 * 256 = 384 → PNG 저장
읽을 때: PNG 값 / 256 = 깊이값 (m)
```

---

### `create_depth_colormap_image(depth_map, output_path)`

**위치**: 라인 681-761

**역할**: 깊이 맵을 컬러 이미지로 시각화

```python
입력:
  depth_map: np.ndarray (float32)
  output_path: Path

출력:
  colored_depth: PNG 이미지
  # JET 컬러맵 적용, 0 픽셀은 흰색
```

**컬러 매핑**:
```
깊이값 정규화 (0 ~ max_depth) → 0~255
0~255 → JET 컬러맵 적용
결과 PNG 저장
```

---

### `run_integrated_pipeline()`

**위치**: 라인 977-1324

**역할**: 전체 파이프라인 오케스트레이션

**단계별 실행**:

```python
for each PCD file:
    1. find_nearest_road_point_and_generate_synthetic_pcd()
       → closest_line_points, original_points
    
    2. build_c_circles_points_from_cloud(original_points)
       → synthetic_points (검은 고리)
    
    3. Merge: points_to_use = original + synthetic
    
    4. 1920×1536 해상도로 투영
       └─ depth_orig, depth_synth, depth_merged
    
    5. [Optional] 640×384 해상도로도 투영
    
    6. 저장
       ├─ newest_pcd/*.pcd (합성된 PCD)
       ├─ newest_depth_maps/*.png (깊이맵)
       ├─ newest_colormap/*.png (컬러)
       ├─ newest_viz_results/*.png (분석)
       ├─ newest_synthetic_depth_maps/*.png (합성만)
       └─ diff_results/*.png (Diff 비교)
    
    7. 640x384_newest/* 도 동일 구조로 저장
```

---

## 실행 예시

### 터미널 입력
```bash
cd c:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation

python integrated_pcd_depth_pipeline_newest.py \
    --parent_folder ncdb-cls-sample/synced_data \
    --ground_z_min -0.95 \
    --ground_z_max 0.5
```

### 콘솔 출력 (예)
```
=== Integrated PCD-to-Depth Pipeline (Closest-Line Mode) ===
Parent folder: ncdb-cls-sample/synced_data
Closest-line parameters:
  - Ground Z range: [-0.950, 0.500]
  - XY distance range: [1.000, 10.000]

[DEBUG] Output base_dir: ncdb-cls-sample/synced_data
  - newest_pcd: ncdb-cls-sample/synced_data/newest_pcd
  - newest_depth_maps: ncdb-cls-sample/synced_data/newest_depth_maps
  - newest_viz_results: ncdb-cls-sample/synced_data/newest_viz_results
  - newest_colormap: ncdb-cls-sample/synced_data/newest_colormap
  - newest_synthetic_depth_maps: ncdb-cls-sample/synced_data/newest_synthetic_depth_maps
  - diff_results: ncdb-cls-sample/synced_data/diff_results
[640x384] Processing 640x384 resolution outputs

Processing 123 PCD files...
 10%|██        | 12/123 [00:45<06:30, 2.85s/it]
  [PROCESS] Starting 0000000931.pcd...
  [SKIP] All outputs (incl. color diff) already exist for 0000000931.
  ...
=== Pipeline Complete ===
Successfully processed: 123 files
Failed: 0 files
Output directories:
  - Closest-line PCDs: ncdb-cls-sample/synced_data/newest_pcd
  - Raw depth maps (16bit): ncdb-cls-sample/synced_data/newest_depth_maps
  - Analysis plots: ncdb-cls-sample/synced_data/newest_viz_results
  - Colorized images: ncdb-cls-sample/synced_data/newest_colormap
  - Diff (merged/synth/orig): ncdb-cls-sample/synced_data/diff_results
[640x384] All 640x384 outputs saved to: ncdb-cls-sample/synced_data/640x384_newest
```

---

## 640×512 추가 생성하기

만약 **640×512** 해상도도 추가로 생성하고 싶다면:

1. `run_integrated_pipeline()` 함수에서 640×384 로직을 참고
2. `resized_image_size = (640, 512)` 추가
3. 동일하게 `project_cloud_to_depth_map_with_labels()` 호출
4. `640x512_newest/` 폴더에 저장

현재는 **test_640x384_div_comparison.py**에서 3가지 해상도(1920×1536, 640×512, 640×384)를 비교하고 있습니다.

---

## 주요 특징

### ✨ 스킵 메커니즘
- 이미 생성된 출력이 있으면 자동 스킵 (중복 방지)
- `[SKIP]` 메시지로 표시

### 🎨 다양한 출력
- **PCD**: 합성된 포인트 클라우드
- **16-bit PNG**: 원본 깊이 맵 (정밀한 값)
- **컬러맵**: 시각적 이해를 위한 컬러 이미지
- **분석 플롯**: 히스토그램 + 통계
- **Diff**: 원본/합성/병합 비교

### 🔧 해상도 유연성
- 1920×1536 기본 해상도
- 640×384 자동 생성 (선택)
- 쉽게 다른 해상도 추가 가능

### 📊 진행률 표시
- tqdm을 사용한 실시간 진행률
- 예상 소요시간 표시

---

## 문제 해결

### "No PCD files found"
```
원인: pcd/ 폴더가 없음
해결: --parent_folder를 pcd 파일이 있는 폴더로 지정
```

### "Sensor 'a6' not found"
```
원인: DEFAULT_CALIB에 정의되지 않음
해결: 코드 상단에서 DEFAULT_CALIB 확인, 필요시 수정
```

### 깊이 맵이 거의 검정색
```
원인: 포인트가 카메라 뒤쪽(Xc < 0)으로 많음
해결: --ground_z_min, --ground_z_max 조정
```

---

## 참고: 핵심 변수 정의

코드 상단에 정의된 상수들:

```python
# 도로 분석 상수
BLUE_DEPTH_Z = -0.771      # 기준 높이
NUM_C_RADII = 50           # 반경 개수
CIRCLE_SEGS = 512          # 각 반경당 점 수
SKIP_FAR_COUNT = 1         # 멀리 있는 반경 스킵
NEAR_BIAS = 1.2            # 가까운 점 강조
MIN_FIRST_RADIUS = 0.01    # 최소 반경
RADIUS_DISTRIBUTION = "uniform"  # 반경 분포
XY_MIN_SEPARATION = 0.1    # XY 최소 분리 거리

# 보정 데이터
DEFAULT_CALIB = {
    "a6": {
        "model": "vadas",
        "intrinsic": [...],      # k, s, div, ux, uy
        "extrinsic": [...],      # 회전/이동 벡터
        "image_size": [1920, 1536]
    }
}
DEFAULT_LIDAR_TO_WORLD = np.eye(4)  # 항등 행렬
```
