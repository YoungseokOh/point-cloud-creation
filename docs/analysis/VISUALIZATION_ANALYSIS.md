# 기존 Visualization 저장 부분 - 파악 결과

## 📍 핵심 위치 정보

### 1️⃣ Visualization 함수 정의
**위치**: `integrated_pcd_depth_pipeline_newest.py` **라인 639-678**

**함수명**: `create_depth_visualization()`

**기능**: 
```
깊이맵을 받아서:
  ├─ 왼쪽: 깊이맵 이미지 (magma 컬러맵)
  ├─ 오른쪽: 깊이 히스토그램 + 통계
  └─ PNG 저장
```

**코드 구조**:
```python
def create_depth_visualization(depth_map: np.ndarray, output_path: Path, title: str) -> None:
    """Creates and saves a depth map visualization with statistics."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))  # 1행 2열
    
    # ax1: 깊이맵 시각화 (magma 컬러맵)
    ax1.imshow(masked, cmap=cmap)
    
    # ax2: 히스토그램 + 통계
    ax2.hist(valid_depths, bins=50)
    
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
```

---

### 2️⃣ 1920×1536 해상도 Visualization 호출 위치
**위치**: **라인 1276-1280**

```python
# 라인 1276-1280
viz_path = newest_viz_results_dir / f"{stem}_depth_analysis.png"
if not viz_path.exists():
    create_depth_visualization(depth_map, viz_path, f"C-Circles Depth Map - {stem}")
else:
    print(f"[SKIP] Viz already exists: {viz_path.name}")
```

**흐름**:
```
depth_map (1920×1536)
    ↓
create_depth_visualization() 호출
    ↓
newest_viz_results_dir/{stem}_depth_analysis.png 저장
```

**저장 경로 예시**:
```
ncdb-cls-sample/synced_data/
└── newest_viz_results/
    └── 0000000931_depth_analysis.png
```

---

### 3️⃣ 640×384 해상도 Visualization 호출 위치
**위치**: **라인 1215-1217**

```python
# 라인 1215-1217
depth_map_resized = projector.project_cloud_to_depth_map(
    camera_name, points_to_use, resized_image_size
)
create_depth_visualization(depth_map_resized, resized_viz_path, f"C-Circles Depth Map 640x384 - {stem}")
```

**흐름**:
```
640×384로 재투영
    ↓
depth_map_resized (640×384)
    ↓
create_depth_visualization() 호출
    ↓
resized_viz_results_dir/{stem}_depth_analysis.png 저장
```

**저장 경로 예시**:
```
ncdb-cls-sample/synced_data/640x384_newest/
└── newest_viz_results/
    └── 0000000931_depth_analysis.png
```

---

## 🎯 두 호출 지점 비교

| 구분 | 1920×1536 | 640×384 |
|------|----------|--------|
| **라인 번호** | 1276-1280 | 1215-1217 |
| **깊이맵 소스** | `depth_map` | `depth_map_resized` |
| **저장 경로** | `newest_viz_results_dir/` | `resized_viz_results_dir/` |
| **파일명** | `{stem}_depth_analysis.png` | `{stem}_depth_analysis.png` |
| **조건문** | `if not viz_path.exists()` | 조건 없음 (항상 실행) |

---

## 📊 현재 구조도

```
run_integrated_pipeline()
│
├─ 1920×1536 처리 구간 (라인 1094~ 1285)
│  ├─ depth_map 생성
│  ├─ colormap_path 저장 (라인 1282-1286)
│  │
│  └─► [✏️ 대체 위치] viz_path 저장 (라인 1276-1280)
│      ├─ 기존: create_depth_visualization()
│      └─ 신규: create_rgb_with_depth_scatter() 로 변경하고 싶음
│
│
├─ 640×384 처리 구간 (라인 1164~ 1227)
│  ├─ depth_map_resized 생성
│  ├─ create_depth_colormap_image() 호출 (라인 1218)
│  │
│  └─► [✏️ 대체 위치] create_depth_visualization() 호출 (라인 1217)
│      ├─ 기존: create_depth_visualization()
│      └─ 신규: create_rgb_with_depth_scatter() 로 변경하고 싶음
│
│
└─ 나머지 처리...
```

---

## 🔍 대체 전 필요한 것들

### RGB 이미지 로드 필요
**현재 상황**: 본 코드에는 **RGB 이미지를 로드하는 부분이 없음**

**필요 사항**:
- RGB 이미지 경로 지정
- 이미지 로드 (cv2.imread)
- 해상도에 맞게 리사이즈

**참고 코드 위치**: `test_640x384_div_comparison.py` 라인 69-84
```python
rgb_image_original = cv2.imread(str(rgb_image_path))
rgb_image_512 = cv2.resize(rgb_image_original, (640, 512), ...)
rgb_with_depth = create_rgb_with_depth_scatter(rgb_image_512.copy(), depth_map_2, ...)
```

---

## 💡 변경 계획 (미리보기)

### 변경 전 (현재)
```python
# 라인 1276-1280 (1920×1536)
viz_path = newest_viz_results_dir / f"{stem}_depth_analysis.png"
if not viz_path.exists():
    create_depth_visualization(depth_map, viz_path, f"C-Circles Depth Map - {stem}")
else:
    print(f"[SKIP] Viz already exists: {viz_path.name}")
```

### 변경 후 (예상)
```python
# 라인 1276-1280 (1920×1536)
viz_path = newest_viz_results_dir / f"{stem}_depth_analysis.png"
if not viz_path.exists():
    # RGB 이미지 로드
    rgb_image = cv2.imread(str(rgb_image_path))
    
    # RGB + 깊이점 오버레이
    rgb_with_depth = create_rgb_with_depth_scatter(
        rgb_image.copy(), depth_map, point_size=4, max_depth=15.0
    )
    
    # 저장
    cv2.imwrite(str(viz_path), rgb_with_depth)
    print(f"[SAVE] RGB+Depth visualization saved: {viz_path.name}")
else:
    print(f"[SKIP] Viz already exists: {viz_path.name}")
```

---

## 📋 정리 (한눈에)

### 기존 Visualization 저장 위치

| 해상도 | 라인 | 함수 | 저장 경로 |
|--------|------|------|---------|
| **1920×1536** | **1276-1280** | `create_depth_visualization()` | `newest_viz_results_dir/` |
| **640×384** | **1215-1217** | `create_depth_visualization()` | `resized_viz_results_dir/` |

### 변경 대상

```
두 가지 모두 대체 가능:
1️⃣ 1920×1536 버전 (라인 1276-1280)
   └─ RGB + depth_map 오버레이로 변경

2️⃣ 640×384 버전 (라인 1215-1217)
   └─ RGB + depth_map_resized 오버레이로 변경
```

### 필요 추가 정보

```
RGB 이미지 경로:
  - 현재 코드에는 없음
  - test_640x384_div_comparison.py처럼 추가 필요
  - sample_id 기반으로 자동 찾기 가능
  
예: ncdb-cls-sample/synced_data/image_a6/{stem}.jpg
```

---

## ✅ 결론

**변경 위치**: 2곳
1. **라인 1276-1280** (1920×1536 visualization)
2. **라인 1215-1217** (640×384 visualization)

**변경 방식**: 
- 기존 `create_depth_visualization()` 제거
- 새로운 `create_rgb_with_depth_scatter()` 호출로 대체

**추가 작업**:
- RGB 이미지 로드 로직 추가
- 해상도별 리사이즈 적용

**파일명 예시**:
```
기존: {stem}_depth_analysis.png (히스토그램 그래프)
신규: {stem}_depth_analysis.png (RGB + depth scatter)
```

---

준비됐으면 언제든 코드 수정 시작하겠습니다! 🚀
