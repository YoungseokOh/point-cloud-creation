# RGB 리사이즈 저장 부분 파악 결과

## 🔍 결론: **본 코드(integrated_pcd_depth_pipeline_newest.py)에는 RGB 리사이즈 저장이 없음**

---

## 📊 현황 분석

### ❌ integrated_pcd_depth_pipeline_newest.py
- RGB 이미지 로드: **없음**
- RGB 리사이즈: **없음**
- RGB 저장: **없음**

**이유**: 이 파이프라인은 깊이맵만 생성하는 목적이므로 RGB 처리 불필요

---

### ✅ test_640x384_div_comparison.py (참고용)
RGB 처리 부분 있음 (라인 161, 234, 310)

```python
# 라인 161: RGB 로드
rgb_image_original = cv2.imread(str(rgb_image_path))

# 라인 234: 640×512로 리사이즈
rgb_image_512 = cv2.resize(rgb_image_original, (640, 512), interpolation=cv2.INTER_AREA)

# 라인 310: 640×384로 리사이즈
rgb_image_384 = cv2.resize(rgb_image_original, (640, 384), interpolation=cv2.INTER_AREA)
```

---

## 🎯 create_rgb_with_depth_scatter() 함수 상세

**위치**: `test_640x384_div_comparison.py` 라인 79-101

**기능**: RGB 이미지에 깊이점들을 오버레이

**입력**:
```python
rgb_image: np.ndarray          # BGR 포맷 이미지 (H×W×3)
depth_map: np.ndarray          # 깊이맵 (H×W)
point_size: int = 2            # 원의 크기 (픽셀)
max_depth: float = 15.0        # 컬러맵 정규화 범위
```

**출력**:
```python
overlay: np.ndarray            # RGB + depth 오버레이 이미지 (H×W×3)
```

**처리 과정**:
```python
1. RGB 이미지 복사
   overlay = rgb_image.copy()

2. 유효한 깊이값 찾기
   valid_mask = depth_map > 0
   valid_coords = np.argwhere(valid_mask)

3. 깊이값 정규화 및 컬러맵 적용
   depths_normalized = depth_map / max_depth  # 0~1 범위
   깊이값 → JET 컬러맵 적용
   
4. 각 픽셀에 원 그리기
   for (y, x) in valid_coords:
       cv2.circle(overlay, (x, y), point_size, color, -1)

5. 결과 반환
   return overlay
```

---

## 📝 코드 변경 계획 (너가 요청한 것)

### 변경 대상: 2곳

#### 1️⃣ 라인 1276-1280 (1920×1536)

**현재 코드**:
```python
viz_path = newest_viz_results_dir / f"{stem}_depth_analysis.png"
if not viz_path.exists():
    create_depth_visualization(depth_map, viz_path, f"C-Circles Depth Map - {stem}")
else:
    print(f"[SKIP] Viz already exists: {viz_path.name}")
```

**변경될 코드** (신규):
```python
viz_path = newest_viz_results_dir / f"{stem}_depth_analysis.png"
if not viz_path.exists():
    # RGB 이미지 로드 (1920×1536)
    rgb_path = base_dir / "image_a6" / f"{stem}.jpg"
    if not rgb_path.exists():
        rgb_path = base_dir / "image_a6" / f"{stem}.png"
    
    if rgb_path.exists():
        rgb_image = cv2.imread(str(rgb_path))
        rgb_with_depth = create_rgb_with_depth_scatter(
            rgb_image.copy(), depth_map, 
            point_size=4, max_depth=15.0
        )
        cv2.imwrite(str(viz_path), rgb_with_depth)
        print(f"[SAVE] RGB+Depth visualization saved: {viz_path.name}")
    else:
        print(f"[SKIP] RGB image not found: {rgb_path}")
else:
    print(f"[SKIP] Viz already exists: {viz_path.name}")
```

---

#### 2️⃣ 라인 1215-1217 (640×384)

**현재 코드**:
```python
depth_map_resized = projector.project_cloud_to_depth_map(
    camera_name, points_to_use, resized_image_size
)
create_depth_visualization(depth_map_resized, resized_viz_path, f"C-Circles Depth Map 640x384 - {stem}")
```

**변경될 코드** (신규):
```python
depth_map_resized = projector.project_cloud_to_depth_map(
    camera_name, points_to_use, resized_image_size
)

# RGB+Depth 시각화
rgb_path = base_dir / "image_a6" / f"{stem}.jpg"
if not rgb_path.exists():
    rgb_path = base_dir / "image_a6" / f"{stem}.png"

if rgb_path.exists():
    rgb_image_original = cv2.imread(str(rgb_path))
    rgb_image_resized = cv2.resize(
        rgb_image_original, resized_image_size, 
        interpolation=cv2.INTER_AREA
    )
    rgb_with_depth = create_rgb_with_depth_scatter(
        rgb_image_resized.copy(), depth_map_resized,
        point_size=2, max_depth=15.0
    )
    cv2.imwrite(str(resized_viz_path), rgb_with_depth)
    print(f"[SAVE] RGB+Depth visualization (640x384) saved: {resized_viz_path.name}")
else:
    print(f"[SKIP] RGB image not found for resized viz: {rgb_path}")
```

---

## 🔧 필요한 추가 작업

### 1️⃣ create_rgb_with_depth_scatter() 함수 추가

**위치**: `integrated_pcd_depth_pipeline_newest.py` 상단 (save_depth_map 근처)

**코드** (test_640x384_div_comparison.py에서 복사):
```python
def create_rgb_with_depth_scatter(rgb_image: np.ndarray, depth_map: np.ndarray,
                                  point_size: int = 2, max_depth: float = 15.0) -> np.ndarray:
    """Draw depth points on RGB image using OpenCV scatter-style visualization."""
    overlay = rgb_image.copy()
    h, w = depth_map.shape
    
    valid_mask = depth_map > 0
    valid_coords = np.argwhere(valid_mask)
    
    if len(valid_coords) == 0:
        return overlay
    
    depths = depth_map[valid_mask]
    depths_normalized = np.clip(depths / max_depth, 0, 1)
    depths_uint8 = (depths_normalized * 255).astype(np.uint8)
    
    depth_colors_1d = cv2.applyColorMap(depths_uint8.reshape(-1, 1, 1), cv2.COLORMAP_JET)
    depth_colors = depth_colors_1d.reshape(-1, 3)
    
    for (y, x), color in zip(valid_coords, depth_colors):
        color_tuple = (int(color[0]), int(color[1]), int(color[2]))
        cv2.circle(overlay, (int(x), int(y)), point_size, color_tuple, -1)
    
    return overlay
```

---

### 2️⃣ RGB 이미지 경로 규칙

**가정**: RGB 이미지가 있는 위치
```
ncdb-cls-sample/synced_data/
├── pcd/
│   └── 0000000931.pcd
└── image_a6/
    └── 0000000931.jpg (또는 .png)
```

**경로 찾기 로직**:
```python
rgb_path = base_dir / "image_a6" / f"{stem}.jpg"
if not rgb_path.exists():
    rgb_path = base_dir / "image_a6" / f"{stem}.png"
```

---

## 📊 요약표

| 항목 | 1920×1536 | 640×384 |
|------|----------|--------|
| **변경 라인** | 1276-1280 | 1215-1217 |
| **RGB 로드** | 필요 | 필요 |
| **RGB 리사이즈** | 불필요 (1920×1536) | 필요 (→ 640×384) |
| **포인트 크기** | 4 | 2 |
| **저장 폴더** | `newest_viz_results/` | `resized_viz_results/` |
| **파일명** | `{stem}_depth_analysis.png` | `{stem}_depth_analysis.png` |

---

## ✅ 최종 확인사항

**이 코드 변경 후**:
```
✅ RGB + depth 오버레이 시각화 생성
✅ 1920×1536 버전 저장
✅ 640×384 버전 저장 (자동 리사이즈)
✅ 기존 히스토그램 시각화 대체
```

**주의사항**:
```
⚠️ RGB 이미지가 없으면 스킵 (에러 아님)
⚠️ image_a6 폴더 경로 확인 필요
⚠️ 파일명이 PCD와 동일해야 함 ({stem}.jpg/png)
```

---

준비됐으면 코드 변경 시작하겠습니다! 🚀
