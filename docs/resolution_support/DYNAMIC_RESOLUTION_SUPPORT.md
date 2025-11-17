# 640×512 지원 코드 변경 완료 ✅

## 🎯 변경 요약

**640×384만 지원 → 640×512 & 640×384 모두 지원**

하드코딩 제거하고 동적으로 처리하도록 변경했습니다.

---

## 📝 변경 사항

### 1️⃣ 라인 1058-1103: 폴더 초기화 부분 (동적 처리)

**변경 전**:
```python
# 하드코딩된 640x384만 지원
resized_base_dir = base_dir / "640x384_newest"
process_resized = not resized_base_dir.exists()

if process_resized:
    resized_pcd_dir = resized_base_dir / "newest_pcd"
    resized_depth_maps_dir = resized_base_dir / "newest_depth_maps"
    ...
```

**변경 후**:
```python
# 동적으로 여러 해상도 지원
resized_resolutions = [(640, 512), (640, 384)]

resized_dirs = {}  # 해상도별 폴더 경로 저장
process_resized = {}  # 해상도별 생성 여부 저장

for res_size in resized_resolutions:
    res_name = f"{res_size[0]}x{res_size[1]}"
    resized_base_dir = base_dir / f"{res_name}_newest"
    process_resized[res_size] = not resized_base_dir.exists()
    
    if process_resized[res_size]:
        # 해상도별 폴더 구조 생성
        resized_dirs[res_size] = {
            'pcd': resized_base_dir / "newest_pcd",
            'depth_maps': resized_base_dir / "newest_depth_maps",
            'viz_results': resized_base_dir / "newest_viz_results",
            'colormap': resized_base_dir / "newest_colormap",
            'synthetic_depth_maps': resized_base_dir / "newest_synthetic_depth_maps",
            'diff_results': resized_base_dir / "diff_results",
        }
        # 폴더 생성
        for dir_path in resized_dirs[res_size].values():
            dir_path.mkdir(parents=True, exist_ok=True)
```

---

### 2️⃣ 라인 1219-1291: 처리 루프 (동적 루프)

**변경 전**:
```python
# 640x384만 처리
if process_resized:
    resized_image_size = (640, 384)  # 하드코딩
    depth_orig_resized, _ = projector.project_cloud_to_depth_map_with_labels(...)
    ...
```

**변경 후**:
```python
# 모든 해상도에 대해 동적 처리
for resized_image_size in resized_resolutions:
    res_name = f"{resized_image_size[0]}x{resized_image_size[1]}"
    
    if not process_resized[resized_image_size]:
        continue
    
    # 해상도별 처리
    depth_orig_resized, _ = projector.project_cloud_to_depth_map_with_labels(
        camera_name, orig_pts, lbl_orig, resized_image_size
    )
    ...
    
    # 해상도별 폴더에서 경로 가져오기
    dirs = resized_dirs[resized_image_size]
    resized_out_pcd_path = dirs['pcd'] / f"{stem}.pcd"
    resized_merged_depth_path = dirs['depth_maps'] / f"{stem}.png"
    ...
    
    # 포인트 크기 자동 결정
    point_size = 4 if resized_image_size[0] >= 1280 else 2
    
    # RGB+Depth 오버레이 (자동 리사이즈)
    rgb_image_resized = cv2.resize(
        rgb_image_original, resized_image_size,
        interpolation=cv2.INTER_AREA
    )
```

---

## 📊 저장 폴더 구조

### 이제 자동으로 생성됩니다:

```
ncdb-cls-sample/synced_data/
│
├─ 1920×1536 (기본 폴더)
│  ├── newest_pcd/
│  ├── newest_depth_maps/
│  ├── newest_viz_results/        ← RGB+Depth
│  ├── newest_colormap/
│  ├── newest_synthetic_depth_maps/
│  └── diff_results/
│
├─ 640x512_newest/ (NEW!)
│  ├── newest_pcd/
│  ├── newest_depth_maps/
│  ├── newest_viz_results/        ← RGB+Depth (리사이즈)
│  ├── newest_colormap/
│  ├── newest_synthetic_depth_maps/
│  └── diff_results/
│
└─ 640x384_newest/ (기존)
   ├── newest_pcd/
   ├── newest_depth_maps/
   ├── newest_viz_results/        ← RGB+Depth (리사이즈)
   ├── newest_colormap/
   ├── newest_synthetic_depth_maps/
   └── diff_results/
```

---

## ✨ 새로운 기능

### 1️⃣ 동적 해상도 지원
```python
resized_resolutions = [(640, 512), (640, 384)]
```

이 리스트에 해상도를 추가하면 자동으로 지원됩니다!

### 2️⃣ 해상도별 포인트 크기 자동 조정
```python
point_size = 4 if resized_image_size[0] >= 1280 else 2
```

- 1280px 이상: 포인트 크기 4
- 그 이하: 포인트 크기 2

### 3️⃣ 폴더 구조 자동 생성
```python
for dir_path in resized_dirs[res_size].values():
    dir_path.mkdir(parents=True, exist_ok=True)
```

모든 폴더가 자동으로 생성됩니다.

---

## 🔧 해상도 추가 방법

**새로운 해상도를 추가하려면**:

```python
# 라인 1062: 이 리스트에 추가
resized_resolutions = [(640, 512), (640, 384), (320, 240)]  # NEW!
```

그뿐입니다! 나머지는 자동으로 처리됩니다. ✨

---

## 🧪 실행 방법

### 640×512 & 640×384 모두 생성
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --parent_folder ncdb-cls-sample/synced_data
```

**생성되는 폴더**:
```
✅ ncdb-cls-sample/synced_data/newest_viz_results/
   └─ RGB+Depth (1920×1536)

✅ ncdb-cls-sample/synced_data/640x512_newest/newest_viz_results/
   └─ RGB+Depth (640×512, 자동 리사이즈)

✅ ncdb-cls-sample/synced_data/640x384_newest/newest_viz_results/
   └─ RGB+Depth (640×384, 자동 리사이즈)
```

---

## 💡 주요 개선사항

| 항목 | 이전 | 이후 |
|------|------|------|
| **지원 해상도** | 640×384만 | 640×512, 640×384 |
| **코드** | 하드코딩 | 동적 루프 |
| **확장성** | 어려움 | 쉬움 (리스트에 추가) |
| **폴더 생성** | 하드코딩 | 자동 생성 |
| **포인트 크기** | 고정 | 해상도별 자동 조정 |

---

## 📄 코드 구조

```
run_integrated_pipeline()
│
├─ 라인 1058-1103: 폴더 초기화 (동적)
│  ├─ resized_resolutions = [(640, 512), (640, 384)]
│  ├─ for res_size in resized_resolutions:
│  │  └─ 각 해상도의 폴더 구조 생성
│  └─ process_resized = {해상도: 생성여부}
│
├─ PCD 파일 반복
│  └─ 라인 1219-1291: 처리 루프 (동적)
│     ├─ for resized_image_size in resized_resolutions:
│     │  ├─ 깊이맵 생성
│     │  ├─ RGB+Depth 오버레이 (자동 리사이즈)
│     │  └─ 해상도별 폴더에 저장
│     └─ 모든 해상도 자동 처리
│
└─ 파이프라인 종료
```

---

## ✅ 테스트 체크리스트

- [ ] 코드 실행: `python integrated_pcd_depth_pipeline_newest.py --parent_folder ncdb-cls-sample/synced_data`
- [ ] 640x512_newest 폴더 생성됨
- [ ] 640x384_newest 폴더 생성됨
- [ ] 각 폴더에 newest_viz_results 생성됨
- [ ] RGB+Depth 이미지 생성됨
- [ ] 포인트 크기 다름 (640x512: 2px vs 1920x1536: 4px)

---

## 🎉 완료!

**640×512가 이제 하드코딩 없이 동적으로 지원됩니다!**

다른 해상도를 추가하고 싶으면 `resized_resolutions` 리스트에만 추가하면 됩니다. 😊
