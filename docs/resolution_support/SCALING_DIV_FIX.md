# Scaling 및 DIV 파라미터 수정 완료

## 🎯 문제점 (수정 전)

### Issue 1: 원본 카메라 모델 수정 문제
```python
# 첫 번째 함수 (project_cloud_to_depth_map_with_labels)
camera_model.scale_intrinsics(scale_x, scale_y)  # ❌ 원본 모델 직접 수정!
```

**문제**: 같은 해상도로 여러 번 호출하면 scaling이 누적됨
```
첫 호출 (640x512): scale = 0.333
  ux = 960 * 0.333 = 320

두 번째 호출 (640x512): scale = 0.333 (하지만 ux는 이미 320!)
  ux = 320 * 0.333 = 106.4  ← 잘못됨!
```

### Issue 2: div 파라미터 스케일링 누락
```python
# 이전 코드의 주석
# div parameter remains constant across different resolutions  ❌ 잘못됨!
```

**문제**: div는 픽셀 좌표 공간의 왜곡 계수이므로 해상도에 따라 스케일링되어야 함
```
원본 (1920×1536): div = D
640×512 (1/3 크기): div = D * (1/3)

왜? div는 픽셀 좌표에서의 왜곡이므로
    해상도가 1/3이면 div도 1/3이어야 정확함
```

---

## ✅ 해결 방법

### Fix 1: 임시 모델 사용 (라인 583-596)
```python
# 이전
camera_model.scale_intrinsics(scale_x, scale_y)  # 원본 수정

# 수정됨
temp_camera_model = VADASFisheyeCameraModel(
    camera_model.original_intrinsic, 
    image_size=(image_width, image_height)
)
temp_camera_model.scale_intrinsics(scale_x, scale_y)
camera_model = temp_camera_model  # ✅ 임시 모델 사용
```

**효과**: 원본 모델은 보존되고, 각 호출마다 독립적으로 스케일링됨

### Fix 2: div 파라미터 스케일링 (라인 488-499)
```python
# 이전
self.div = self.original_intrinsic[8]  # 스케일링 안 함

# 수정됨
avg_scale = (scale_x + scale_y) / 2.0
self.div = self.original_intrinsic[8] * avg_scale  # ✅ 스케일링
```

**효과**: 640×512, 640×384 등 모든 해상도에서 정확한 왜곡 계수 적용

---

## 📊 Intrinsic 파라미터 구조

```
intrinsic = [
    k[0], k[1], k[2], k[3], k[4], k[5], k[6],  # [0:7]   = k (다항식 계수)
    s,                                           # [7]     = s
    div,                                         # [8]     = div (왜곡) ← 이제 스케일됨
    ux,                                          # [9]     = ux (주점 X) ← 스케일됨
    uy                                           # [10]    = uy (주점 Y) ← 스케일됨
]
```

**스케일링 규칙**:
```
원본 (W₀, H₀)  →  타겟 (W, H)

scale_x = W / W₀
scale_y = H / H₀

스케일링 필요:
  ✅ ux = ux₀ * scale_x      (주점 X는 가로 해상도에 비례)
  ✅ uy = uy₀ * scale_y      (주점 Y는 세로 해상도에 비례)
  ✅ div = div₀ * avg_scale  (왜곡은 평균 스케일에 비례)
  ❌ k                       (다항식 계수는 정규화된 좌표에서만 작동)
  ❌ s                       (크기 파라미터는 스케일하지 않음)
```

---

## 🧮 예시: 640×512 (1/3 축소)

### 원본 (1920×1536)
```
ux_orig = 960
uy_orig = 768
div_orig = 15.0
```

### 640×512로 축소
```
scale_x = 640 / 1920 = 0.333
scale_y = 512 / 1536 = 0.333
avg_scale = 0.333

스케일링 결과:
  ux = 960 * 0.333 = 320
  uy = 768 * 0.333 = 256
  div = 15.0 * 0.333 = 5.0  ← 이제 올바름!
```

---

## ✨ 최종 효과

| 항목 | 이전 | 이후 |
|------|------|------|
| **원본 모델 보존** | ❌ 누적되는 오류 | ✅ 항상 정확 |
| **DIV 스케일링** | ❌ 미적용 | ✅ 적용됨 |
| **다중 호출** | ❌ 오류 누적 | ✅ 독립적 처리 |
| **640×512 정확도** | ❌ 부정확 | ✅ 정확 |

---

## 🔍 검증 방법

640×512로 생성된 깊이맵을 1920×1536 이미지와 비교하면:
- **이전**: 포인트 위치 오류 발생
- **이후**: 포인트 정렬 정확함

이는 스케일링된 카메라 파라미터가 올바르게 적용되었음을 의미합니다.
