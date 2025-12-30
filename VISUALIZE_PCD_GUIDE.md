# visualize_pcd.py - PCD를 이미지에 투영하는 고급 도구

## 개요

`visualize_pcd.py`는 다음 기능을 제공합니다:

1. **PCD 파일 읽기**: Binary 및 ASCII 포맷 모두 지원
2. **카메라 투영**: VADAS Fisheye 카메라 모델 사용
3. **이미지 오버레이**: 깊이값을 색상으로 표시
4. **3D 시각화**: Open3D를 이용한 3D 포인트 클라우드 보기

---

## 사용법

### 기본 실행

```bash
python visualize_pcd.py
```

### 코드에서 직접 사용

```python
from visualize_pcd import (
    load_pcd_xyz,
    project_cloud_to_image,
    CalibrationDB,
    DEFAULT_CALIB,
    DEFAULT_LIDAR_TO_WORLD_v3
)
import cv2
from pathlib import Path

# 1. PCD 파일 로드
pcd_path = Path("ncdb-cls-sample/synced_data/pcd/0000000931.pcd")
cloud_xyz = load_pcd_xyz(pcd_path)

# 2. 이미지 로드
image_path = Path("ncdb-cls-sample/synced_data/image_a6/0000000931.jpg")
image = cv2.imread(str(image_path))

# 3. 보정 데이터 설정
calib_db = CalibrationDB(DEFAULT_CALIB, lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v3)

# 4. 투영
output_image, in_front, on_image = project_cloud_to_image(
    cloud_xyz=cloud_xyz,
    image=image,
    calib_db=calib_db,
    camera_name="a6",
    max_distance=50.0,
    point_radius=3
)

# 5. 결과 저장
cv2.imwrite("output.jpg", output_image)
```

---

## 주요 클래스 및 함수

### 클래스 1: VADASFisheyeCameraModel

VADAS Polynomial Fisheye 카메라 모델

```python
camera = VADASFisheyeCameraModel(
    intrinsic=[k0, k1, k2, k3, k4, k5, k6, focal, pixel_size, cx, cy],
    image_size=(1920, 1536)
)

# 투영
u, v, valid = camera.project_point(Xc, Yc, Zc)
# Xc, Yc, Zc: 카메라 좌표
# u, v: 이미지 좌표 (픽셀)
# valid: 이미지 범위 내 여부
```

**Intrinsic 파라미터**:
- `k0~k6`: Polynomial 계수 (7개)
- `focal`: 초점 거리
- `pixel_size`: 픽셀 크기
- `cx, cy`: 주점 (principal point)

**투영 공식**:
```
1. 극좌표 변환: nx = -Yc, ny = -Zc
2. 거리: rho = dist / Xc
3. Polynomial: rd = k0 + k1*rho + k2*rho² + ... + k6*rho⁶
4. 각도: cosPhi = nx/dist, sinPhi = ny/dist
5. 픽셀: u = rd*cosPhi*focal/pixel_size + cx
         v = rd*sinPhi*focal/pixel_size + cy
```

### 클래스 2: CalibrationDB

카메라 보정 데이터 관리

```python
calib_db = CalibrationDB(
    calib_dict=DEFAULT_CALIB,
    lidar_to_world=DEFAULT_LIDAR_TO_WORLD_v3
)

# 카메라 정보 조회
camera_info = calib_db.get("a6")
# → {
#     "model": VADASFisheyeCameraModel,
#     "extrinsic": 변환 행렬 (4x4),
#     "intrinsic": 파라미터 리스트
# }
```

### 함수 1: load_pcd_xyz()

PCD 파일에서 XYZ 좌표 추출

```python
points = load_pcd_xyz(Path("0000000931.pcd"))
# 반환: (N, 3) 배열
#   points[:, 0] = X
#   points[:, 1] = Y
#   points[:, 2] = Z
```

**지원 포맷**:
- Binary PCD (POINT_DATA binary)
- ASCII PCD (POINT_DATA ascii)

### 함수 2: project_cloud_to_image()

포인트 클라우드를 이미지에 투영

```python
output_image, in_front_count, on_image_count = project_cloud_to_image(
    cloud_xyz=cloud_xyz,              # (N, 3) 포인트 배열
    image=image,                      # 입력 이미지 (BGR, uint8)
    calib_db=calib_db,                # 보정 데이터베이스
    camera_name="a6",                 # 카메라 이름
    max_distance=50.0,                # 최대 표시 거리 (미터)
    point_radius=3                    # 포인트 원의 반경 (픽셀)
)
```

**반환값**:
- `output_image`: 투영된 포인트가 그려진 이미지
- `in_front_count`: 카메라 앞의 포인트 수
- `on_image_count`: 이미지 범위 내 포인트 수

### 함수 3: get_color_from_distance()

거리에 따른 색상 계산 (Jet colormap)

```python
color = get_color_from_distance(distance=10.0, max_distance=50.0)
# 반환: (B, G, R) 튜플
#   0m: 파란색
#   ~12.5m: 초록색
#   ~25m: 노란색
#   ~37.5m: 주황색
#   50m+: 빨간색
```

### 함수 4: visualize_pcd_3d()

Open3D를 이용한 3D 시각화

```python
visualize_pcd_3d(Path("0000000931.pcd"))
```

**요구 사항**:
```bash
pip install open3d
```

---

## 좌표계 변환 흐름

```
LiDAR 좌표 (World)
    ↓
[lidar_to_world 행렬 곱하기]
    ↓
World 좌표
    ↓
[extrinsic 행렬 곱하기]
    ↓
Camera 좌표 (Xc, Yc, Zc)
    ↓
[VADAS 투영 공식]
    ↓
이미지 좌표 (u, v)
```

**코드**:
```python
cloud_xyz_hom = np.hstack([cloud_xyz, np.ones((cloud_xyz.shape[0], 1))])
lidar_to_camera = extrinsic @ calib_db.lidar_to_world
cam_pts_hom = (lidar_to_camera @ cloud_xyz_hom.T).T
cam_pts = cam_pts_hom[:, :3]  # (Xc, Yc, Zc)
```

---

## 보정 데이터 (Calibration)

### DEFAULT_CALIB 구조

```python
DEFAULT_CALIB = {
    "a6": {
        "model": "vadas",
        "intrinsic": [k0, k1, k2, k3, k4, k5, k6, focal, pixel_size, cx, cy],
        "extrinsic": [rx, ry, rz, tx, ty, tz],  # Rodrigues 벡터
        "image_size": None  # 런타임에 설정
    }
}
```

### DEFAULT_LIDAR_TO_WORLD_v3

ref_calibration_data.py에서 가져온 LiDAR to World 변환 행렬

```python
DEFAULT_LIDAR_TO_WORLD_v3 = np.array([
    [-0.99856, -0.00901632, -0.052883, 0.0539789],
    [0.00872567, -0.999946, 0.00572437, 0.0837919],
    [-0.0529318, 0.00525468, 0.998584, 0.737086],
    [0., 0., 0., 1.]
])
```

---

## 실행 결과

### 출력 예시

```
======================================================================
PCD를 이미지에 투영하기
======================================================================

[1] 파일 확인
  ✓ PCD: ncdb-cls-sample/synced_data/pcd/0000000931.pcd
  ✓ 이미지: ncdb-cls-sample/synced_data/image_a6/0000000931.jpg

[2] PCD 파일 로드
  포인트 수: 50000
  데이터 포맷: binary
  ✓ 로드 완료: 50000 포인트

[3] 이미지 로드
  ✓ 로드 완료: 1920×1536

[4] 카메라 보정 설정
  ✓ 카메라: a6
  ✓ Fisheye 모델

[5] 포인트 클라우드 투영
  투영 중 (50000 포인트)...
  카메라 앞: 45000개
  이미지 범위: 38000개
  ✓ 완료

[6] 결과 저장
  ✓ 저장 완료: output/0000000931_projected.jpg

[7] 통계
  총 포인트: 50000
  카메라 앞: 45000 (90.0%)
  이미지 범위: 38000 (76.0%)

[8] 추가 시각화
  3D 시각화를 하시겠습니까? (y/n):
======================================================================
```

---

## 설정 커스터마이징

### 스크립트 수정

`visualize_pcd.py` 파일의 `main()` 함수 시작 부분:

```python
def main():
    # ===== 설정 =====
    data_root = Path("ncdb-cls-sample/synced_data")  # 데이터 경로
    pcd_filename = "0000000931"                       # PCD 파일명
    camera_name = "a6"                                # 카메라 이름
    max_distance = 50.0                               # 최대 표시 거리 (미터)
    point_radius = 3                                  # 포인트 크기 (픽셀)
```

### 색상 방식 변경

`get_color_from_distance()` 함수를 수정하여 다른 colormap 사용 가능:

```python
# 현재: Jet colormap
# 변경 가능: Viridis, Hot, Cool 등

# 또는 OpenCV의 colormap 사용
def get_color_from_distance_cv(distance, max_distance=50.0):
    normalized = min(distance / max_distance, 1.0)
    color_idx = int(normalized * 255)
    # cv2.COLORMAP_JET, cv2.COLORMAP_VIRIDIS 등
```

---

## 문제 해결

### 문제 1: "ModuleNotFoundError: No module named 'open3d'"

```bash
# 해결: open3d 설치
pip install open3d
```

### 문제 2: 포인트가 이미지에 표시되지 않음

**확인 사항**:
1. PCD 파일 경로 확인
2. 이미지 경로 확인
3. 보정 데이터 (extrinsic, intrinsic) 확인
4. 좌표계 정의 확인

**디버깅**:
```python
# 투영 전 통계
print(f"포인트 범위: {cloud_xyz.min(axis=0)} ~ {cloud_xyz.max(axis=0)}")
print(f"카메라 앞 포인트: {in_front}/{len(cloud_xyz)}")
print(f"이미지 범위 포인트: {on_image}/{len(cloud_xyz)}")
```

### 문제 3: 포인트가 이미지의 한쪽에만 표시됨

**원인**: Extrinsic 행렬 또는 좌표계 정의가 잘못됨

**해결**:
1. extrinsic 벡터 확인
2. lidar_to_world 행렬 확인
3. 카메라 좌표축 정의 확인 (X forward, Y left, Z up)

---

## 고급 사용법

### 1. 여러 PCD 파일 일괄 처리

```python
from pathlib import Path

pcd_dir = Path("ncdb-cls-sample/synced_data/pcd")
for pcd_file in sorted(pcd_dir.glob("*.pcd"))[:100]:  # 처음 100개
    print(f"처리 중: {pcd_file.name}")
    
    cloud_xyz = load_pcd_xyz(pcd_file)
    image = cv2.imread(str(pcd_file.stem + ".jpg"))
    
    output_image, _, _ = project_cloud_to_image(cloud_xyz, image, calib_db)
    cv2.imwrite(f"output/{pcd_file.stem}_proj.jpg", output_image)
```

### 2. 깊이 범위 필터링

```python
# 특정 깊이 범위의 포인트만 표시
filtered_cloud = cloud_xyz[(cloud_xyz[:, 0] > 0) & (cloud_xyz[:, 0] < 50)]

output_image, _, _ = project_cloud_to_image(filtered_cloud, image, calib_db)
```

### 3. 포인트 원 대신 사각형 그리기

```python
# project_cloud_to_image() 함수의 cv2.circle() 부분을 수정
# cv2.circle(output_image, (u, v), point_radius, color, -1)
# ↓
# cv2.rectangle(output_image, (u-r, v-r), (u+r, v+r), color, -1)
```

---

## 성능 팁

| 작업 | 시간 | 메모리 |
|------|------|--------|
| 50K 포인트 로드 | ~100ms | ~3MB |
| 투영 | ~500ms | ~5MB |
| 이미지 저장 | ~100ms | 변수 |
| **총** | **~700ms** | **~8MB** |

### 최적화 방법

1. **포인트 필터링**: 범위 밖의 포인트 제거
2. **해상도 축소**: 더 작은 이미지 사용
3. **배치 처리**: 여러 파일을 한 번에 처리

---

## 관련 파일

- `ref_calibration_data.py`: 보정 데이터 정의
- `ref_camera_lidar_projector.py`: 투영 알고리즘 참고
- `integrated_pcd_depth_pipeline_newest.py`: 더 고급 기능

---

