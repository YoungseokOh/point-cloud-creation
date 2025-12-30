# 투영 코드 검증 보고서

**작성일**: 2025-11-25  
**검증 대상**: `visualize_pcd.py`의 VADAS Fisheye 투영 공식  
**상태**: ✅ **수정 완료**

---

## 1. 문제점 분석

### 1.1 원래 코드의 문제

원래 `visualize_pcd.py`의 투영 공식:

```python
# ❌ 잘못된 방식
rho = dist / Xc if Xc > 0 else 0
rd = self.k[0]
for i in range(1, len(self.k)):
    rd += self.k[i] * (rho ** i)

u = u_norm * self.focal / self.pixel_size + self.cx
v = v_norm * self.focal / self.pixel_size + self.cy
```

**문제점**:
1. ❌ `theta = atan2(dist, Xc)` 단계가 누락됨 (CRITICAL!)
2. ❌ Intrinsic 파라미터 해석이 완전히 잘못됨:
   - 사용한 파라미터: `focal`, `pixel_size`, `cx`, `cy`
   - 실제 파라미터: `s` (focal scale), `div` (normalization), `ux`, `uy` (principal point)
3. ❌ Polynomial 평가 방식이 잘못됨 (간단한 산술이 아님)
4. ❌ 이미지 중심 오프셋 계산 누락

---

## 2. 정확한 VADAS 투영 공식 (검증됨)

### 2.1 참조 구현

**출처**:
- `integrated_pcd_depth_pipeline_newest.py` (메인 파이프라인, 사용 중)
- `test_640x384_div_comparison.py` (검증된 테스트)
- `ref_camera_lidar_projector.py` (원본 구현)

### 2.2 정확한 투영 공식

```python
# ✅ 올바른 방식

# 1단계: 극좌표 변환 (VADAS 규약)
nx = -Yc
ny = -Zc
dist = sqrt(nx² + ny²)
cosPhi = nx / dist
sinPhi = ny / dist

# 2단계: 각도 계산 (CRITICAL!)
theta = atan2(dist, Xc)

# 3단계: Polynomial 입력
xd = theta * s

# 4단계: Polynomial 평가 (Horner's method)
rd = poly_eval(k, xd) / div

# 5단계: 이미지 좌표
img_w_half = image_size[0] / 2
img_h_half = image_size[1] / 2
u = rd * cosPhi * scale_x + ux + img_w_half
v = rd * sinPhi * scale_y + uy + img_h_half
```

### 2.3 Intrinsic 파라미터 (11개)

```
intrinsic[0:7]   = k[0~6]    # Polynomial coefficients (u2d)
intrinsic[7]     = s         # Focal length scale
intrinsic[8]     = div       # Normalization divisor (CRITICAL!)
intrinsic[9]     = ux        # Principal point X offset
intrinsic[10]    = uy        # Principal point Y offset
```

#### 예시 값 (a6 카메라):
```python
intrinsic = [
    -0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,  # k[0:7]
    1.0447,    # s
    0.0021,    # div (⚠️ CRITICAL - 매우 작은 값!)
    44.9516,   # ux
    2.48822    # uy
]
```

---

## 3. 주요 수정 사항

### 3.1 VADASFisheyeCameraModel 클래스

**변경 전**:
- 잘못된 파라미터 해석 (focal, pixel_size 사용)
- 간단한 산술 공식
- theta 계산 누락

**변경 후**:
- 정확한 파라미터 해석 (s, div, ux, uy)
- theta를 이용한 복잡한 계산
- `_poly_eval()` 메서드로 Horner's method 사용
- Aspect ratio scaling 지원 (scale_x, scale_y)

### 3.2 Intrinsic 파라미터 업데이트

```python
DEFAULT_CALIB = {
    "a6": {
        "intrinsic": [
            # k0~k6: 7개
            -0.0004, 1.0136, -0.0623, 0.2852, -0.332, 0.1896, -0.0391,
            # s, div, ux, uy: 4개
            1.0447,    # s
            0.0021,    # div
            44.9516,   # ux
            2.48822    # uy
        ],
        "image_size": (1920, 1536)
    }
}
```

---

## 4. 검증 기준

### 4.1 코드 비교

| 항목 | 올바른 구현 | visualize_pcd.py |
|------|-----------|-----------------|
| theta 계산 | ✅ `atan2(dist, Xc)` | ✅ 수정됨 |
| Polynomial 평가 | ✅ `poly / div` | ✅ `_poly_eval()` 사용 |
| 이미지 중심 | ✅ `img_w_half` 포함 | ✅ 수정됨 |
| Intrinsic 파라미터 | ✅ 11개 (s, div, ux, uy) | ✅ 수정됨 |
| Aspect ratio | ✅ `scale_x, scale_y` | ✅ 지원 |

### 4.2 테스트 코드

- ✅ `test_visualize_pcd.py` 존재 (5개 테스트)
- ✅ `test_640x384_div_comparison.py` (검증된 비교 테스트)
- ✅ `verify_depth_rgb_alignment.py` (정렬 검증)

---

## 5. 좌표계 검증

### 5.1 VADAS 좌표계

```
카메라 시점:
    +X: 정면 (forward)
    +Y: 오른쪽 (right)
    +Z: 아래쪽 (down)

극좌표 변환 (VADAS 규약):
    nx = -Yc   (왼쪽을 양수로)
    ny = -Zc   (위쪽을 양수로)
```

### 5.2 좌표 변환 체인

```
LiDAR 점군 (world frame)
    ↓ [lidar_to_world 행렬]
World 좌표
    ↓ [extrinsic 행렬]
Camera 좌표 (Xc, Yc, Zc)
    ↓ [VADAS 투영]
이미지 좌표 (u, v)
```

**코드**:
```python
cloud_xyz_hom = np.hstack([cloud_xyz, np.ones((cloud_xyz.shape[0], 1))])
lidar_to_camera = extrinsic @ calib_db.lidar_to_world
cam_pts_hom = (lidar_to_camera @ cloud_xyz_hom.T).T
cam_pts = cam_pts_hom[:, :3]  # Camera coordinates
```

---

## 6. 참조 파일 비교

### 6.1 integrated_pcd_depth_pipeline_newest.py

```python
# 올바른 구현 (라인 511-544)
theta = math.atan2(dist, Xc)           # ✅
xd = theta * self.s                     # ✅
rd = self._poly_eval(self.k, xd) / self.div  # ✅
u = rd * cosPhi * self.scale_x + self.ux + img_w_half  # ✅
```

### 6.2 visualize_pcd.py (수정 후)

```python
# 동일한 구현 (라인 129-172)
theta = math.atan2(dist, Xc)           # ✅
xd = theta * self.s                     # ✅
rd = self._poly_eval(self.k, xd) / self.div  # ✅
u = rd * cosPhi * self.scale_x + self.ux + img_w_half  # ✅
```

---

## 7. 추가 개선사항

### 7.1 주석 강화

- VADAS 모델에 대한 자세한 설명 추가
- Intrinsic 파라미터 의미 설명
- 각 단계별 계산 과정 문서화

### 7.2 에러 처리

- NaN/Inf 검사: `if math.isinf(rd) or math.isnan(rd)`
- Division by zero 방지: `if abs(self.div) < 1e-9`
- Extreme value 처리: `if dist < 1e-10`

### 7.3 Extrinsic 행렬 업데이트

원래:
```python
"extrinsic": [0.293769, -0.0542026, -0.631615, ...]  # 구버전
```

수정 후:
```python
"extrinsic": [0.119933, -0.129544, -0.54216, ...]    # ref_calibration_data.py 권장값
```

---

## 8. 결론

### ✅ 수정 완료

`visualize_pcd.py`의 투영 코드는 다음과 같이 수정되었습니다:

1. **Theta 계산 추가**: `atan2(dist, Xc)` 사용
2. **Intrinsic 파라미터 수정**: s, div, ux, uy로 변경
3. **Polynomial 평가**: Horner's method로 구현
4. **이미지 중심 오프셋**: 올바르게 계산
5. **Aspect ratio 지원**: scale_x, scale_y 구현

### 🧪 검증 방법

```bash
# 테스트 실행
python test_visualize_pcd.py

# 실제 투영
python visualize_pcd.py
```

### 📊 예상 결과

- 카메라 앞의 포인트가 정확히 이미지에 투영됨
- 거리에 따른 색상 표시 (Jet colormap)
- 깊이맵 생성 시 정확한 값

---

## 9. 참고 자료

- `integrated_pcd_depth_pipeline_newest.py`: 메인 파이프라인 (신뢰도 ⭐⭐⭐⭐⭐)
- `test_640x384_div_comparison.py`: 검증된 테스트 (신뢰도 ⭐⭐⭐⭐⭐)
- `ref_calibration_data.py`: 캘리브레이션 데이터 (신뢰도 ⭐⭐⭐⭐)
- `verify_depth_rgb_alignment.py`: 정렬 검증 (신뢰도 ⭐⭐⭐⭐)

