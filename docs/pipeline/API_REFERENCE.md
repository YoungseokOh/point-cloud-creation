# integrated_pcd_depth_pipeline_newest.py - API 상세 레퍼런스

## 📚 목차

1. **클래스 API**
   - VADASFisheyeCameraModel
   - CalibrationDB
   - LidarCameraProjector

2. **함수 API**
   - PCD 파일 처리
   - 깊이맵 생성
   - 시각화

3. **데이터 형식**

---

## 🏗️ VADASFisheyeCameraModel 클래스

### 역할
VADAS Fisheye 카메라의 3D→2D 투영을 담당하는 핵심 클래스

### 초기화

```python
camera = VADASFisheyeCameraModel(
    intrinsic=[k0, k1, k2, k3, k4, k5, k6, s, div, ux, uy],
    image_size=(1920, 1536),
    camera_matrix=[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
)
```

**파라미터**:

| 파라미터 | 타입 | 설명 | 예시 |
|---------|------|------|------|
| `intrinsic` | list[11] | VADAS Fisheye intrinsic | [0.8, 0.2, 0.1, ...] |
| `image_size` | tuple(2) | 이미지 해상도 (W, H) | (1920, 1536) |
| `camera_matrix` | array(3,3) | 카메라 내부 파라미터 | [[fx, 0, cx], ...] |

**Intrinsic 파라미터 상세**:

```python
intrinsic[0] = k0      # 다항식 계수 1
intrinsic[1] = k1      # 다항식 계수 2
intrinsic[2] = k2      # 다항식 계수 3
intrinsic[3] = k3      # 다항식 계수 4
intrinsic[4] = k4      # 다항식 계수 5
intrinsic[5] = k5      # 다항식 계수 6
intrinsic[6] = k6      # 다항식 계수 7 (7개 총 다항식)
intrinsic[7] = s       # 크기 파라미터
intrinsic[8] = div     # 왜곡 계수 (절대 스케일링 금지!)
intrinsic[9] = ux      # 주점 X (스케일링해야 함)
intrinsic[10] = uy     # 주점 Y (스케일링해야 함)
```

### 메서드 1: scale_intrinsics()

```python
camera.scale_intrinsics(target_image_size=(640, 512))
```

**목적**: 특정 해상도로 카메라 파라미터 스케일링

**파라미터**:
- `target_image_size` (tuple): 목표 해상도 (W, H)

**동작**:
1. 원본 해상도와 목표 해상도 비율 계산
   - `scale_x = target_w / original_w`
   - `scale_y = target_h / original_h`

2. 파라미터 업데이트
   - `ux_new = ux * scale_x`
   - `uy_new = uy * scale_y`
   - `div` → **변경 없음** (중요!)

3. 내부 변수 저장
   - `self.scale_x = scale_x`
   - `self.scale_y = scale_y`

**예시**:
```python
# 원본: 1920×1536, 목표: 640×512
camera.scale_intrinsics((640, 512))
# scale_x = 640/1920 = 0.333
# scale_y = 512/1536 = 0.333
```

### 메서드 2: project_point()

```python
u, v, valid = camera.project_point(cx, cy, cz)
```

**목적**: 카메라 좌표계의 3D 점을 이미지 좌표로 투영

**파라미터**:
- `cx` (float): 카메라 좌표 X (오른쪽 방향)
- `cy` (float): 카메라 좌표 Y (아래 방향)
- `cz` (float): 카메라 좌표 Z (앞 방향, 깊이)

**반환값**:
- `u` (float): 이미지 X 좌표 (픽셀)
- `v` (float): 이미지 Y 좌표 (픽셀)
- `valid` (bool): 이미지 범위 내인지 여부

**투영 공식** (상세):

```python
# Step 1: 카메라 좌표에서 거리와 각도
cx_norm = cx / norm(cx, cy, cz)
cy_norm = cy / norm(cx, cy, cz)

# Step 2: 극좌표 거리
rho = sqrt(cx_norm^2 + cy_norm^2)

# Step 3: 다항식 거리
rd = k[0] + k[1]*rho + k[2]*rho^2 + k[3]*rho^3 + \
     k[4]*rho^4 + k[5]*rho^5 + k[6]*rho^6

# Step 4: 각도 성분 (스케일 적용!)
cosPhi = cx_norm / sqrt(cx_norm^2 + cy_norm^2)
sinPhi = cy_norm / sqrt(cx_norm^2 + cy_norm^2)

# Step 5: 이미지 좌표 (scale_x, scale_y 적용!)
u = rd * cosPhi * self.scale_x + self.ux + img_w_half
v = rd * sinPhi * self.scale_y + self.uy + img_h_half
```

**예시**:
```python
# 1m 앞, 정면 중앙
u, v, valid = camera.project_point(0, 0, 1.0)
# valid = True
# u, v = 대략 이미지 중앙

# 범위 밖 (뒤쪽)
u, v, valid = camera.project_point(0, 0, -1.0)
# valid = False
```

### 속성 (Properties)

```python
camera.intrinsic        # 현재 intrinsic 파라미터
camera.original_intrinsic  # 스케일링 전 원본
camera.image_size       # 현재 이미지 크기
camera.original_image_size  # 원본 이미지 크기
camera.scale_x          # X 스케일 인자
camera.scale_y          # Y 스케일 인자
```

---

## 📊 CalibrationDB 클래스

### 역할
여러 카메라의 보정 정보를 관리

### 초기화

```python
calib_db = CalibrationDB(calibration_file_path="calibration.json")
```

### 메서드: get_camera_model()

```python
camera_model = calib_db.get_camera_model(
    camera_name="camera_0",
    image_size=(1920, 1536)
)
```

**파라미터**:
- `camera_name` (str): 카메라 이름
- `image_size` (tuple): 원본 이미지 해상도

**반환값**:
- `VADASFisheyeCameraModel` 인스턴스

---

## 🎯 LidarCameraProjector 클래스

### 역할
3D 포인트 클라우드를 이미지에 투영하여 깊이맵 생성

### 초기화

```python
projector = LidarCameraProjector(calibration_db)
```

### 메서드 1: project_cloud_to_depth_map_with_labels()

```python
depth_map, provenance_map = projector.project_cloud_to_depth_map_with_labels(
    camera_name="camera_0",
    points=np.array([[x1, y1, z1], [x2, y2, z2], ...]),
    labels=np.array([0, 0, 1, 1, ...]),
    image_size=(1920, 1536)
)
```

**파라미터**:

| 파라미터 | 타입 | 설명 | 예시 |
|---------|------|------|------|
| `camera_name` | str | 카메라 이름 | "camera_0" |
| `points` | ndarray(N, 3) | 3D 포인트 배열 | np.array([...]) |
| `labels` | ndarray(N,) | 각 포인트의 라벨 | np.array([0, 1, ...]) |
| `image_size` | tuple(2) | 깊이맵 해상도 | (1920, 1536) |

**반환값**:

| 반환값 | 타입 | 설명 |
|-------|------|------|
| `depth_map` | ndarray(H, W) float32 | 깊이 (미터) |
| `provenance_map` | ndarray(H, W) int8 | 포인트 출처 (0=원본, 1=합성) |

**동작 흐름**:

```python
# Step 1: 깊이맵 초기화 (0으로 채움)
depth_map = zeros(image_size)
provenance_map = zeros(image_size)

# Step 2: 각 포인트 투영
for each point in points:
    # LiDAR 좌표 → 카메라 좌표 (extrinsic 적용)
    cx, cy, cz = lidar_to_camera(point)
    
    # 카메라 좌표 → 이미지 좌표
    u, v, valid = camera.project_point(cx, cy, cz)
    
    if valid:
        # Step 3: 깊이값 저장 (가장 가까운 포인트만)
        if cz < depth_map[v, u]:  # 기존값보다 가까우면
            depth_map[v, u] = cz
            provenance_map[v, u] = labels[point_idx]

# Step 4: 반환
return depth_map, provenance_map
```

**예시**:
```python
# 원본 포인트만
orig_pts = np.array([[x1, y1, z1], [x2, y2, z2], ...])
orig_labels = np.zeros(len(orig_pts), dtype=int)  # 모두 0

depth_orig, prov_orig = projector.project_cloud_to_depth_map_with_labels(
    "camera_0", orig_pts, orig_labels, (1920, 1536)
)

# 합성 포인트만
synth_pts = np.array([[...], [...], ...])
synth_labels = np.ones(len(synth_pts), dtype=int)  # 모두 1

depth_synth, prov_synth = projector.project_cloud_to_depth_map_with_labels(
    "camera_0", synth_pts, synth_labels, (1920, 1536)
)

# 병합
merged_pts = np.vstack([orig_pts, synth_pts])
merged_labels = np.concatenate([orig_labels, synth_labels])

depth_merge, prov_merge = projector.project_cloud_to_depth_map_with_labels(
    "camera_0", merged_pts, merged_labels, (1920, 1536)
)
```

### 메서드 2: project_cloud_to_depth_map()

```python
depth_map = projector.project_cloud_to_depth_map(
    camera_name="camera_0",
    points=np.array([[x1, y1, z1], ...]),
    image_size=(1920, 1536)
)
```

**차이점**: `labels` 파라미터 없음, `provenance_map` 반환 안함

---

## 🛠️ 유틸리티 함수

### PCD 파일 처리

#### parse_pcd_fallback()

```python
points = parse_pcd_fallback(pcd_file_path)
```

**목적**: Binary 또는 ASCII PCD 파일 읽기

**파라미터**:
- `pcd_file_path` (str): PCD 파일 경로

**반환값**:
- `ndarray(N, 6)`: [x, y, z, intensity, t, ring]

**지원 형식**:
- Binary PCD (POINT_DATA BINARY)
- ASCII PCD (POINT_DATA ASCII)

**예시**:
```python
points = parse_pcd_fallback("./output/0000000000.pcd")
print(f"로드된 포인트: {points.shape[0]}")  # 50000
print(f"좌표 범위: {points[:, :3].min()}~{points[:, :3].max()}")
```

#### save_synthetic_pcd()

```python
save_synthetic_pcd(points, output_path)
```

**목적**: PCD 파일로 저장

**파라미터**:
- `points` (ndarray): 포인트 배열
- `output_path` (str): 저장 경로

### 깊이맵 처리

#### save_depth_map()

```python
save_depth_map(output_path, depth_map)
```

**목적**: 깊이맵을 uint16 PNG로 저장 (KITTI 포맷)

**파라미터**:
- `output_path` (str): 저장 경로
- `depth_map` (ndarray float32): 깊이값 (미터)

**변환 공식**:
```python
# 저장
depth_uint16 = uint16(depth_map * 256)  # float → uint16

# 읽기
depth_meters = depth_uint16.astype(float) / 256.0
```

**예시**:
```python
save_depth_map("output/0000000000.png", depth_map)

# 검증
import cv2
loaded = cv2.imread("output/0000000000.png", cv2.IMREAD_UNCHANGED)
depth_loaded = loaded.astype(np.float32) / 256.0
```

#### create_rgb_with_depth_scatter()

```python
rgb_with_depth = create_rgb_with_depth_scatter(
    rgb_image=rgb_bgr,
    depth_map=depth_meters,
    point_size=2,
    max_depth=15.0
)
```

**목적**: RGB 이미지에 깊이값을 색상으로 표시

**파라미터**:

| 파라미터 | 타입 | 설명 | 기본값 |
|---------|------|------|--------|
| `rgb_image` | ndarray(H, W, 3) uint8 | RGB 이미지 (BGR) | 필수 |
| `depth_map` | ndarray(H, W) float32 | 깊이맵 (미터) | 필수 |
| `point_size` | int | 포인트 크기 (픽셀) | 2 |
| `max_depth` | float | 최대 표시 깊이 (미터) | 15.0 |

**색상 맵**:
```
Depth Range          Color
─────────────────────────
0.0m (검정)         없음 (배경)
0.5m ~ 2.0m         초록색 (아주 가까움)
2.0m ~ 5.0m         초록-노란색
5.0m ~ 10.0m        노란색 (중간)
10.0m ~ 15.0m       빨강색 (먼것)
15.0m+ (빨강)       붉은색 (아주 멀음)
```

**반환값**:
- `ndarray(H, W, 3) uint8`: RGB 오버레이 이미지

**예시**:
```python
rgb = cv2.imread("image.jpg")  # BGR로 읽힘
depth = cv2.imread("depth.png", cv2.IMREAD_UNCHANGED).astype(np.float32) / 256.0
rgb_depth = create_rgb_with_depth_scatter(rgb, depth, point_size=4, max_depth=20.0)
cv2.imwrite("output.png", rgb_depth)
```

#### create_depth_colormap_image()

```python
create_depth_colormap_image(
    depth_map=depth_meters,
    output_path="output/colormap.png",
    colormap_type="jet"
)
```

**목적**: 깊이맵을 Colormap으로 시각화

**파라미터**:
- `depth_map` (ndarray): 깊이값
- `output_path` (str): 저장 경로
- `colormap_type` (str): Colormap 유형

**Colormap 유형**:
```
"jet"      → 파란색→초록색→노란색→빨강색
"viridis"  → 보라색→초록색→노란색
"hot"      → 검정색→빨강색→노란색→흰색
"cool"     → 시안→마젠타
```

---

## 📊 데이터 형식

### 입력 형식: calibration.json

```json
{
  "cameras": {
    "camera_0": {
      "intrinsic": [0.8, 0.2, 0.1, -0.05, 0.01, -0.002, 0.0001, 1.2, 0.5, 960, 768],
      "extrinsic": {
        "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "translation": [0, 0, 0]
      },
      "image_size": [1920, 1536],
      "camera_matrix": [[1200, 0, 960], [0, 1200, 768], [0, 0, 1]]
    }
  }
}
```

### 출력 형식: 깊이맵 (PNG)

```
파일 형식: uint16 PNG
값 범위: 0 ~ 65535
변환: depth_meters = uint16_value / 256.0

예시:
  5.0m  → 5.0 * 256 = 1280 → PNG에 저장
  10.2m → 10.2 * 256 = 2611 → PNG에 저장
  0m    → 0 → PNG에 저장 (포인트 없음)
```

### 포인트 클라우드 형식: PCD

```
Binary PCD 형식:
├─ Header
│  ├─ VERSION 0.7
│  ├─ FIELDS x y z intensity t ring
│  ├─ SIZE 4 4 4 4 4 4
│  ├─ TYPE f f f f f f
│  ├─ COUNT 1 1 1 1 1 1
│  ├─ WIDTH <N>
│  ├─ HEIGHT 1
│  ├─ VIEWPOINT 0 0 0 1 0 0 0
│  ├─ POINTS <N>
│  └─ DATA binary
│
└─ Data
   ├─ float32 x[0], y[0], z[0], intensity[0], t[0], ring[0]
   ├─ float32 x[1], y[1], z[1], intensity[1], t[1], ring[1]
   └─ ...
```

---

## 🔄 통합 예제

### 전체 워크플로우

```python
import numpy as np
from integrated_pcd_depth_pipeline_newest import (
    CalibrationDB,
    LidarCameraProjector,
    parse_pcd_fallback,
    save_depth_map,
    create_rgb_with_depth_scatter
)
import cv2

# Step 1: 보정 데이터 로드
calib_db = CalibrationDB("calibration.json")

# Step 2: Projector 생성
projector = LidarCameraProjector(calib_db)

# Step 3: PCD 파일 로드
points = parse_pcd_fallback("0000000000.pcd")
print(f"로드된 포인트: {points.shape}")

# Step 4: 깊이맵 생성
depth_map, prov_map = projector.project_cloud_to_depth_map_with_labels(
    camera_name="camera_0",
    points=points[:, :3],  # x, y, z만 사용
    labels=np.zeros(len(points), dtype=int),  # 모두 원본
    image_size=(1920, 1536)
)
print(f"깊이맵: {depth_map.shape}")
print(f"깊이 범위: {depth_map.min():.2f}m ~ {depth_map.max():.2f}m")

# Step 5: 깊이맵 저장
save_depth_map("output_depth.png", depth_map)

# Step 6: RGB+Depth 시각화
rgb = cv2.imread("image.jpg")
rgb_depth = create_rgb_with_depth_scatter(rgb, depth_map, point_size=2, max_depth=15.0)
cv2.imwrite("output_viz.png", rgb_depth)

print("완료!")
```

---

## 📈 성능 최적화 팁

### 메모리 효율
```python
# 배치 처리 (전체 로드하지 않음)
batch_size = 100
for i in range(0, len(points), batch_size):
    batch = points[i:i+batch_size]
    depth_batch, _ = projector.project_cloud_to_depth_map_with_labels(
        camera_name, batch, labels[i:i+batch_size], image_size
    )
```

### 속도 최적화
```python
# 포인트 전처리 (범위 밖 제거)
valid_mask = (points[:, 2] > 0) & (points[:, 2] < 50)  # z > 0, z < 50m
points_valid = points[valid_mask]
labels_valid = labels[valid_mask]

# 깊이맵 생성
depth_map, _ = projector.project_cloud_to_depth_map_with_labels(
    camera_name, points_valid, labels_valid, image_size
)
```

