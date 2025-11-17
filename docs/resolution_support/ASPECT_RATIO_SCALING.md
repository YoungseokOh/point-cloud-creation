# Aspect Ratio Scaling Implementation

## 📐 개요

VADAS Fisheye 카메라 모델에서 **비균일 종횡비(non-uniform aspect ratio)** 스케일링을 지원하기 위한 구현 문서입니다.

---

## 🔑 핵심 원리

### **기존 방식 (잘못됨)**

```python
# ❌ div를 스케일링하는 방식
camera_model.div = original_div / scale_x
u = rd * cosPhi + ux
v = rd * sinPhi + uy
```

**문제점:**
- `div`는 정규화된 각도 공간에서 작동하는 파라미터
- `div`를 스케일링하면 640×384와 같은 비균일 종횡비에서 왜곡 발생
- scale_x와 scale_y가 다를 때 정확한 투영 불가

### **새로운 방식 (올바름)**

```python
# ✅ div는 원본 유지, aspect ratio를 최종 좌표에 적용
camera_model.div = original_div  # 원본 유지!
camera_model.scale_x = scale_x   # 저장
camera_model.scale_y = scale_y   # 저장

u = rd * cosPhi * scale_x + ux
v = rd * sinPhi * scale_y + uy
```

**장점:**
- `div`는 정규화 파라미터로 유지
- `scale_x`, `scale_y`를 최종 좌표에만 적용
- 비균일 종횡비 (640×384) 정확히 지원

---

## 💻 코드 구현

### 1️⃣ **`__init__` - 초기화**

```python
def __init__(self, intrinsic: List[float], image_size: Optional[Tuple[int, int]] = None):
    # ... 기존 코드 ...
    self.original_intrinsic = intrinsic.copy()
    self.scale_x = 1.0  # ← 추가
    self.scale_y = 1.0  # ← 추가
```

### 2️⃣ **`scale_intrinsics` - 스케일링 설정**

```python
def scale_intrinsics(self, scale_x: float, scale_y: float) -> None:
    """Scale intrinsic parameters for different image sizes
    
    - ux, uy: 이미지 크기에 따라 스케일
    - div: 원본 유지! (정규화 파라미터)
    - scale_x, scale_y: 저장 후 project_point()에서 사용
    """
    # Principal point 스케일링
    self.ux = self.original_intrinsic[9] * scale_x
    self.uy = self.original_intrinsic[10] * scale_y
    
    # div는 원본 유지!
    self.div = self.original_intrinsic[8]
    
    # Scale factors 저장
    self.scale_x = scale_x
    self.scale_y = scale_y
```

### 3️⃣ **`project_point` - 투영**

```python
def project_point(self, Xc: float, Yc: float, Zc: float) -> Tuple[int, int, bool]:
    # ... 정규화, 각도 계산 ...
    
    rd = self._poly_eval(self.k, xd) / self.div  # div는 원본!
    
    # Aspect ratio를 최종 좌표에 적용
    u = rd * cosPhi * self.scale_x + self.ux + img_w_half
    v = rd * sinPhi * self.scale_y + self.uy + img_h_half
    
    return int(round(u)), int(round(v)), True
```

---

## 📊 해상도별 비교

| 해상도 | scale_x | scale_y | 특징 | div 처리 |
|--------|---------|---------|------|---------|
| **1920×1536** | 1.0 | 1.0 | 원본 (스케일 없음) | 원본 유지 |
| **640×512** | 0.333 | 0.333 | 균일 스케일 (정사각형 픽셀) | 원본 유지 |
| **640×384** | 0.333 | 0.250 | **비균일 스케일** (종횡비 왜곡) | 원본 유지 |

### **640×512 예시**

```python
# 스케일 팩터
scale_x = 640 / 1920 = 0.333
scale_y = 512 / 1536 = 0.333

# 인트린직 스케일링
ux = original_ux * 0.333
uy = original_uy * 0.333
div = original_div  # 원본 유지!

# 투영
rd = poly(theta) / div
u = rd * cosPhi * 0.333 + ux + 320
v = rd * sinPhi * 0.333 + uy + 256
```

### **640×384 예시**

```python
# 스케일 팩터 (비균일!)
scale_x = 640 / 1920 = 0.333
scale_y = 384 / 1536 = 0.250

# 인트린직 스케일링
ux = original_ux * 0.333
uy = original_uy * 0.250  # ← 다름!
div = original_div  # 원본 유지!

# 투영
rd = poly(theta) / div
u = rd * cosPhi * 0.333 + ux + 320
v = rd * sinPhi * 0.250 + uy + 192  # ← scale_y 다름!
```

---

## 🧪 검증 방법

### **test_640x384_div_comparison.py**

```python
# 1920×1536 (원본)
depth_map_1 = project_manually(points, ..., scale_x=1.0, scale_y=1.0)

# 640×512 (균일)
depth_map_2 = project_manually(points, ..., scale_x=0.333, scale_y=0.333)

# 640×384 (비균일)
depth_map_3 = project_manually(points, ..., scale_x=0.333, scale_y=0.250)
```

**실행:**
```bash
python test_640x384_div_comparison.py
```

**출력:**
- 세 가지 해상도의 RGB+Depth 비교 이미지
- Coverage, 평균 깊이 통계
- 시각적 검증 가능

---

## ✅ 체크리스트

- [x] `__init__`에 `scale_x`, `scale_y` 추가
- [x] `scale_intrinsics`에서 `div` 원본 유지
- [x] `project_point`에서 aspect ratio 적용
- [x] 640×512 검증 (균일 스케일)
- [x] 640×384 검증 (비균일 스케일)
- [x] 1920×1536 정상 작동 확인

---

## 📚 참고

- `ref_camera_lidar_projector.py` - 원본 투영 로직
- `test_640x384_div_comparison.py` - 검증 스크립트
- `integrated_pcd_depth_pipeline_newest.py` - 메인 파이프라인

---

## 🎯 결론

**div는 정규화 파라미터이므로 원본을 유지하고, aspect ratio는 최종 픽셀 좌표에만 적용합니다.**

이를 통해:
- ✅ 비균일 종횡비 (640×384) 정확히 지원
- ✅ 균일 종횡비 (640×512) 정상 작동
- ✅ 원본 해상도 (1920×1536) 영향 없음

**모든 해상도에서 정확한 깊이맵 생성이 보장됩니다!** ✨
