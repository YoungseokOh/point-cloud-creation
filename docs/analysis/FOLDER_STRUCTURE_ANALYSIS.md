# 640×512 저장 폴더 분석 - 코드 확인 결과

## 🔍 결론: **640×512는 별도 폴더로 저장되지 않음** ❌

현재 코드는 **640×384만** 별도 폴더로 저장합니다.

---

## 📊 현재 저장 구조

### 1️⃣ 1920×1536 해상도
```
ncdb-cls-sample/synced_data/
├── newest_pcd/
├── newest_depth_maps/
├── newest_viz_results/          ← 변경됨 (RGB+Depth 오버레이)
├── newest_colormap/
├── newest_synthetic_depth_maps/
└── diff_results/
```

**특징**: 기본 폴더, 항상 생성됨

---

### 2️⃣ 640×384 해상도 (별도 폴더)
```
ncdb-cls-sample/synced_data/
└── 640x384_newest/              ← 별도 폴더
    ├── newest_pcd/
    ├── newest_depth_maps/
    ├── newest_viz_results/      ← 변경됨 (RGB+Depth 오버레이, 리사이즈됨)
    ├── newest_colormap/
    ├── newest_synthetic_depth_maps/
    └── diff_results/
```

**특징**: 640x384_newest 폴더가 있으면 자동 스킵

---

## 🔧 핵심 코드 분석

### 라인 1060-1061: 폴더 존재 여부 결정
```python
resized_base_dir = base_dir / "640x384_newest"
process_resized = not resized_base_dir.exists()
```

**의미**:
```
process_resized = True   ← 640x384_newest 폴더 없음 → 생성할 예정
process_resized = False  ← 640x384_newest 폴더 있음 → 스킵
```

---

### 라인 1063-1075: 폴더 구조 생성
```python
if process_resized:
    resized_pcd_dir = resized_base_dir / "newest_pcd"
    resized_depth_maps_dir = resized_base_dir / "newest_depth_maps"
    resized_viz_results_dir = resized_base_dir / "newest_viz_results"
    resized_colormap_dir = resized_base_dir / "newest_colormap"
    resized_synthetic_depth_maps_dir = resized_base_dir / "newest_synthetic_depth_maps"
    resized_diff_results_dir = resized_base_dir / "diff_results"
    
    # 모든 폴더 생성
    resized_pcd_dir.mkdir(parents=True, exist_ok=True)
    resized_depth_maps_dir.mkdir(parents=True, exist_ok=True)
    # ...
```

**의미**: 640x384_newest 폴더 구조 자동 생성

---

### 라인 1207: 해상도 고정값
```python
if process_resized:
    resized_image_size = (640, 384)  ← 하드코딩된 값
```

**의미**: 항상 640×384로 고정 (640×512 불가능)

---

## 📂 640×512 추가 시 필요한 변경

### 현재 상태
```
640×384 전용 폴더 생성 코드
└─ 640x384_newest/ 만 지원
```

### 640×512 추가하려면
```
✅ 별도 폴더 이름 필요: 640x512_newest/
✅ 조건부 로직 필요: process_resized_512 변수
✅ 새로운 경로 변수들 필요: resized_512_*_dir
✅ 해상도 변경: resized_image_size = (640, 512)
```

---

## 🎯 현재 코드 흐름도

```
run_integrated_pipeline() 시작
│
├─ 라인 1060-1061: 640x384_newest 폴더 확인
│  └─ 없으면 process_resized = True
│
├─ 라인 1063-1075: 폴더 구조 생성 (if process_resized)
│
├─ PCD 파일 반복 처리
│  └─ 라인 1206-1276: 640×384 생성 (if process_resized)
│     ├─ 라인 1207: resized_image_size = (640, 384)
│     ├─ 라인 1209-1215: 깊이맵 생성 (640×384)
│     ├─ 라인 1218-1276: 640x384_newest/ 에 저장
│     └─ 라인 1240-1262: RGB+Depth 오버레이 (640×384 리사이즈)
│
└─ 파이프라인 종료
```

---

## 📊 저장 경로 비교표

| 항목 | 1920×1536 | 640×384 | 640×512 |
|------|----------|--------|--------|
| **폴더명** | 기본 폴더 | `640x384_newest/` | ❌ 없음 |
| **코드 지원** | ✅ 기본 | ✅ 있음 | ❌ 없음 |
| **별도 저장** | 아니오 | 예 | N/A |
| **RGB+Depth** | ✅ | ✅ | N/A |
| **RGB 리사이즈** | 없음 | 자동 | N/A |

---

## ⚠️ 640×512 생성 현황

### 현재
```
❌ 640×512 해상도 미지원
❌ 별도 폴더 저장 불가능
```

### 640×512가 필요하면
```
옵션 1: test_640x384_div_comparison.py 사용
        └─ 이미 640×512 비교 기능 있음
        
옵션 2: 본 코드 수정 필요
        ├─ 폴더명: 640x512_newest/
        ├─ 해상도: (640, 512)
        └─ 조건부 로직 추가 필요
```

---

## 🔍 코드 위치 정확히

### 폴더 정의 (라인 1060-1075)
```python
# 라인 1060: 폴더명 정의
resized_base_dir = base_dir / "640x384_newest"

# 라인 1061: 생성 여부 결정
process_resized = not resized_base_dir.exists()

# 라인 1063-1075: 폴더 구조 생성
if process_resized:
    resized_pcd_dir = resized_base_dir / "newest_pcd"
    resized_depth_maps_dir = resized_base_dir / "newest_depth_maps"
    ...
```

### 해상도 정의 (라인 1207)
```python
# 라인 1206-1207: 조건부 640×384 처리
if process_resized:
    resized_image_size = (640, 384)  ← 하드코딩
```

### 저장 경로 (라인 1218-1276)
```python
# 라인 1218-1221: 저장 경로 정의
resized_out_pcd_path = resized_pcd_dir / f"{stem}.pcd"
resized_merged_depth_path = resized_depth_maps_dir / f"{stem}.png"
resized_viz_path = resized_viz_results_dir / f"{stem}_depth_analysis.png"
...

# 라인 1225-1276: 실제 저장
save_synthetic_pcd(points_to_use, resized_out_pcd_path)
save_depth_map(resized_merged_depth_path, depth_merge_resized)
...
```

---

## 📌 결론

### 현재 상황
```
✅ 1920×1536 저장 (기본 폴더)
✅ 640×384 저장 (640x384_newest/ 폴더)
❌ 640×512 저장 (미지원)
```

### 640×512 추가 방법
```
1. test_640x384_div_comparison.py 사용 (권장)
   └─ 이미 구현되어 있음
   
2. 본 코드 수정 (복잡함)
   ├─ 폴더명 추가: 640x512_newest/
   ├─ 조건 추가: process_resized_512
   └─ 해상도 변경: (640, 512)
```

### 지금 당장 실행하려면
```
640×384만 생성됨:
python integrated_pcd_depth_pipeline_newest.py \
    --parent_folder ncdb-cls-sample/synced_data
```

---

## 📄 참고

**640×512 비교 시각화**:
```bash
python test_640x384_div_comparison.py 0000000931
```

**출력**: 1920×1536 | 640×512 | 640×384 3가지 해상도 비교 이미지
